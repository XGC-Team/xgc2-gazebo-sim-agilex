#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <xgc2_math/control/wheel_contact_kinematics.hpp>

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace scout_gazebo {

// Read-only planar wheel diagnostic. No gain, force, velocity or pose writes.
// Separate wheel-target error from rolling/sliding error before tuning anything.
class WheelDiagnostics : public gazebo::ModelPlugin {
 public:
  ~WheelDiagnostics() override {
    update_.reset();
    for (auto& subscription : subscriptions_) subscription.shutdown();
    queue_.disable();
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    if (!ros::isInitialized()) throw std::runtime_error("Scout diagnostics requires gazebo_ros");
    model_ = model;
    nh_.reset(new ros::NodeHandle(sdf->Get<std::string>("robotNamespace")));
    nh_->setCallbackQueue(&queue_);
    mode_ = nh_->advertise<std_msgs::String>("simulation/dynamics_mode", 1, true);
    std_msgs::String mode;
    mode.data = "wheel_physics";
    mode_.publish(mode);
    bool enabled = false;
    nh_->param("wheel_diagnostics/enabled", enabled, false);
    if (!enabled) return;
    radius_ = sdf->Get<double>("wheelRadius");
    nh_->param("wheel_diagnostics/publish_rate", rate_, 50.0);
    if (!std::isfinite(radius_) || radius_ <= 0.0 || !std::isfinite(rate_) || rate_ <= 0.0)
      throw std::runtime_error("Invalid Scout wheel diagnostics geometry/rate");
    base_ = model_->GetLink("base_link");
    if (!base_) throw std::runtime_error("Scout diagnostics requires base_link");
    const std::array<std::string, 4> names{{"front_left", "front_right", "rear_left", "rear_right"}};
    const std::array<std::string, 4> controllers{{"fl", "fr", "rl", "rr"}};
    for (std::size_t i = 0; i < names.size(); ++i) {
      wheels_[i] = model_->GetJoint(names[i] + "_wheel");
      const auto link = model_->GetLink(names[i] + "_wheel_link");
      if (!wheels_[i] || !link || link->GetCollisions().empty())
        throw std::runtime_error("Scout diagnostics missing wheel joint/collision: " + names[i]);
      collisions_[i] = link->GetCollision(0u);
      subscriptions_[i] = nh_->subscribe<std_msgs::Float64>(
          "scout_motor_" + controllers[i] + "_controller/command", 1,
          [this, i](const std_msgs::Float64::ConstPtr& command) {
            targets_[i] = command->data;
          });
    }
    publisher_ = nh_->advertise<std_msgs::Float64MultiArray>("simulation/wheel_kinematics", 1);
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&WheelDiagnostics::Update, this, std::placeholders::_1));
  }

 private:
  void Update(const gazebo::common::UpdateInfo& info) {
    const double time = info.simTime.Double();
    if (time < last_seen_) {
      targets_.fill(std::numeric_limits<double>::quiet_NaN());
      last_publish_ = time - 1.0 / rate_;
      queue_.clear();
    }
    last_seen_ = time;
    queue_.callAvailable(ros::WallDuration(0));
    if (time - last_publish_ < 1.0 / rate_ - 1e-12) return;
    last_publish_ = time;
    const auto frame = base_->WorldPose();
    const auto cog = base_->WorldCoGPose().Pos();
    const auto linear = frame.Rot().RotateVectorReverse(base_->WorldCoGLinearVel());
    const auto angular = frame.Rot().RotateVectorReverse(base_->WorldAngularVel());
    std_msgs::Float64MultiArray message;
    message.layout.dim.resize(2);
    message.layout.dim[0].label = "wheel:front_left,front_right,rear_left,rear_right";
    message.layout.dim[0].size = 4;
    message.layout.dim[0].stride = 48;
    message.layout.dim[1].label = "sim_time_s,target_rad_s,measured_rad_s,joint_effort_nm,x_m,y_m,rolling_m_s,ground_x_m_s,ground_y_m_s,slip_x_m_s,slip_y_m_s,axis_body_y";
    message.layout.dim[1].size = 12;
    message.layout.dim[1].stride = 12;
    message.data.reserve(48);
    for (std::size_t i = 0; i < wheels_.size(); ++i) {
      // Use collision centres, not mounting joints; Scout's tyre midpoint is
      // offset outwards along the axle. Flat-ground, planar approximation only.
      const auto position = frame.Rot().RotateVectorReverse(collisions_[i]->WorldPose().Pos() - cog);
      const auto axis = frame.Rot().RotateVectorReverse(wheels_[i]->GlobalAxis(0));
      const double measured = wheels_[i]->GetVelocity(0);
      const double rolling = radius_ * measured * axis.Y();
      const auto contact = xgc2_math::wheelContactKinematics(
          linear.X(), linear.Y(), angular.Z(), position.X(), position.Y(), rolling);
      for (double value : {time, targets_[i], measured, wheels_[i]->GetForce(0), position.X(), position.Y(), rolling,
                           contact.longitudinal_ground_m_s, contact.lateral_ground_m_s,
                           contact.longitudinal_slip_m_s, contact.lateral_slip_m_s, axis.Y()})
        message.data.push_back(value);
    }
    publisher_.publish(message);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_;
  std::array<gazebo::physics::JointPtr, 4> wheels_;
  std::array<gazebo::physics::CollisionPtr, 4> collisions_;
  std::array<ros::Subscriber, 4> subscriptions_;
  std::array<double, 4> targets_{{NAN, NAN, NAN, NAN}};
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue queue_;
  ros::Publisher publisher_, mode_;
  gazebo::event::ConnectionPtr update_;
  double radius_{0.08}, rate_{50.0}, last_publish_{-1.0}, last_seen_{0.0};
};
GZ_REGISTER_MODEL_PLUGIN(WheelDiagnostics)
}  // namespace scout_gazebo
