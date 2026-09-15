#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <xgc2_math/control/delayed_planar_velocity.hpp>
#include <xgc2_math/control/wheel_drive.hpp>

#include "xgc_chassis_hold/udp.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace scout_gazebo {

// Planar kinematic plant for algorithm development, not a traction/contact model.
// Gazebo still integrates poses and collisions; this plugin never teleports poses.
class UnicyclePlugin : public gazebo::ModelPlugin {
 public:
  ~UnicyclePlugin() override {
    update_.reset();
    subscription_.shutdown();
    running_.store(false);
    queue_.disable();
    if (thread_.joinable()) thread_.join();
    if (gate_) xgc_chassis_hold::Hub::instance().remove(gate_.get());
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    if (!ros::isInitialized()) throw std::runtime_error("Scout unicycle requires gazebo_ros");
    model_ = model;
    base_ = model_->GetLink("base_link");
    if (!base_) throw std::runtime_error("Scout unicycle requires base_link");
    const std::string ns = sdf->Get<std::string>("robotNamespace");
    nh_.reset(new ros::NodeHandle(ns));
    nh_->setCallbackQueue(&queue_);
    ros::NodeHandle parameters(*nh_, "unicycle");
    xgc2_math::DelayedPlanarVelocityParameters p;
    parameters.param("command_delay_s", p.delay_s, 0.005);
    parameters.param("linear_time_constant_s", p.linear_time_constant_s, 0.005);
    parameters.param("angular_time_constant_s", p.yaw_time_constant_s, 0.005);
    // This mode deliberately has lag; reject accidental ideal-model settings.
    if (!(p.linear_time_constant_s > 0.0 && p.yaw_time_constant_s > 0.0))
      throw std::runtime_error("Scout unicycle time constants must be positive");
    drive_.reset(new xgc2_math::DelayedPlanarVelocity(p));
    parameters.param("enable_command_limits", limits_, true);
    parameters.param("max_linear_speed", max_v_, 1.5);
    parameters.param("max_angular_speed", max_w_, 1.0);
    parameters.param("publish_rate", publish_rate_, 50.0);
    radius_ = sdf->Get<double>("wheelRadius");
    track_ = sdf->Get<double>("wheelTrack");
    for (double value : {max_v_, max_w_, publish_rate_, radius_, track_}) {
      if (!std::isfinite(value) || value <= 0.0)
        throw std::runtime_error("Scout unicycle limits, geometry and publish rate must be finite and positive");
    }
    joints_ = nh_->advertise<sensor_msgs::JointState>("joint_states", 1);
    velocity_ = nh_->advertise<geometry_msgs::TwistStamped>("simulation/drive/velocity", 1);
    mode_ = nh_->advertise<std_msgs::String>("simulation/dynamics_mode", 1, true);
    now_ = model_->GetWorld()->SimTime().Double();
    last_publish_ = now_ - 1.0 / publish_rate_;
    drive_->reset(now_);
    gate_.reset(new xgc_chassis_hold::Gate(xgc_chassis_hold::lastPath(ns)));
    gate_->setZeroThunk(&UnicyclePlugin::HoldZero, this);
    xgc_chassis_hold::Hub::instance().add(gate_.get());
    subscription_ = nh_->subscribe("cmd_vel", 1, &UnicyclePlugin::Command, this);
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&UnicyclePlugin::Update, this, std::placeholders::_1));
    running_.store(true);
    thread_ = std::thread([this] {
      while (running_.load() && nh_->ok()) queue_.callAvailable(ros::WallDuration(0.01));
    });
    std_msgs::String mode;
    mode.data = "unicycle";
    mode_.publish(mode);
    ROS_INFO_STREAM("Scout " << ns << " unicycle: delay=" << p.delay_s
                    << " tau_v=" << p.linear_time_constant_s << " tau_w=" << p.yaw_time_constant_s
                    << "; unity gains; wheel effort controllers disabled");
  }

  void Reset() override {
    if (!gate_) return;
    gate_->withCommand([this](bool) {
      now_ = model_->GetWorld()->SimTime().Double();
      drive_->reset(now_);
      angle_.fill(0.0);
      last_publish_ = now_ - 1.0 / publish_rate_;
      Apply({});  // Gazebo calls Reset on its simulation thread.
      Publish({});
    });
  }

 private:
  static void HoldZero(void* self) {
    auto& plugin = *static_cast<UnicyclePlugin*>(self);
    // Called with Gate held, including while simulation time is frozen.
    // Clear queued commands AND lag states immediately. Gazebo writes remain on
    // its update thread: a paused world cannot move, and resume first writes zero.
    plugin.drive_->reset(plugin.now_);
    plugin.Publish({});
  }

  void Command(const geometry_msgs::Twist::ConstPtr& message) {
    if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
      ROS_WARN_THROTTLE(1.0, "Ignoring nonfinite Scout unicycle command");
      return;
    }
    gate_->withCommand([this, &message](bool held) {
      if (held) return;
      xgc2_math::PlanarVelocity input{message->linear.x, message->angular.z};
      if (limits_) {
        input.linear_m_s = std::clamp(input.linear_m_s, -max_v_, max_v_);
        input.yaw_rad_s = std::clamp(input.yaw_rad_s, -max_w_, max_w_);
      }
      try {
        // now_ is the last observed physics time, not ROS/wall time. Reception
        // has at most one physics-step timestamp quantization in either run mode.
        drive_->command(now_, input);
      } catch (const std::exception& e) {
        drive_->reset(now_);
        Publish({});
        ROS_ERROR_STREAM("Scout unicycle command rejected; drive cleared: " << e.what());
      }
    });
  }

  void Update(const gazebo::common::UpdateInfo& info) {
    gate_->withCommand([this, &info](bool held) {
      const double time = info.simTime.Double();
      const double dt = std::max(0.0, time - now_);
      if (time < now_) {
        angle_.fill(0.0);
        last_publish_ = time - 1.0 / publish_rate_;
      }
      now_ = time;
      if (held) drive_->reset(time);
      const auto state = drive_->advance(time);
      Apply(state);
      const auto wheels = xgc2_math::differentialDriveWheelVelocity(
          state.linear_m_s, state.yaw_rad_s, {radius_, track_, 1.0, 1.0});
      // Presentation/status only. These are explicitly virtual wheel states,
      // never used to produce forces or to report measured tire slip.
      angle_[0] += dt * wheels.left_rad_s;
      angle_[1] += dt * wheels.right_rad_s;
      if (time - last_publish_ >= 1.0 / publish_rate_ - 1e-12) {
        Publish(state);
        last_publish_ = time;
      }
    });
  }

  void Apply(xgc2_math::PlanarVelocity state) {
    const auto reference = base_->WorldPose();
    const double yaw = reference.Rot().Yaw();
    const ignition::math::Vector3d omega(0.0, 0.0, state.yaw_rad_s);
    const ignition::math::Vector3d origin_velocity(
        state.linear_m_s * std::cos(yaw), state.linear_m_s * std::sin(yaw), base_->WorldCoGLinearVel().Z());
    // Assign a CONSISTENT rigid velocity field to every articulated link.
    // Model::SetLinearVel(v) alone gives all link CoGs the same velocity during
    // yaw; joint projection would then manufacture a different chassis response.
    for (const auto& link : model_->GetLinks()) {
      const auto offset = link->WorldCoGPose().Pos() - reference.Pos();
      link->SetLinearVel(origin_velocity + omega.Cross(offset));
      link->SetAngularVel(omega);
    }
  }

  void Publish(xgc2_math::PlanarVelocity state) {
    geometry_msgs::TwistStamped velocity;
    velocity.header.stamp = ros::Time::now();
    velocity.header.frame_id = nh_->getNamespace() + "/base_link";
    velocity.twist.linear.x = state.linear_m_s;
    velocity.twist.angular.z = state.yaw_rad_s;
    velocity_.publish(velocity);
    const auto wheels = xgc2_math::differentialDriveWheelVelocity(
        state.linear_m_s, state.yaw_rad_s, {radius_, track_, 1.0, 1.0});
    sensor_msgs::JointState joints;
    joints.header.stamp = velocity.header.stamp;
    joints.header.frame_id = "unicycle_virtual_wheels";
    joints.name = {"front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"};
    joints.position = {angle_[0], angle_[1], angle_[0], angle_[1]};
    joints.velocity = {wheels.left_rad_s, wheels.right_rad_s, wheels.left_rad_s, wheels.right_rad_s};
    // Empty effort: there is no measured actuator torque in this mode.
    joints_.publish(joints);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_;
  gazebo::event::ConnectionPtr update_;
  std::unique_ptr<ros::NodeHandle> nh_;
  std::unique_ptr<xgc_chassis_hold::Gate> gate_;
  std::unique_ptr<xgc2_math::DelayedPlanarVelocity> drive_;
  ros::CallbackQueue queue_;
  ros::Subscriber subscription_;
  ros::Publisher joints_, velocity_, mode_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  bool limits_{true};
  double now_{0.0}, last_publish_{0.0}, radius_{0.08}, track_{0.416503};
  double max_v_{1.5}, max_w_{1.0}, publish_rate_{50.0};
  std::array<double, 2> angle_{{0.0, 0.0}};
};
GZ_REGISTER_MODEL_PLUGIN(UnicyclePlugin)
}  // namespace scout_gazebo
