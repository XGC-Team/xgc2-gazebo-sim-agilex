#include "scout_gazebo/scout_skid_steer.hpp"
#include <gazebo/common/Events.hh>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wescore {
ScoutSkidSteer::ScoutSkidSteer(ros::NodeHandle* node, gazebo::physics::ModelPtr model, std::string robot_id)
    : robot_id_(std::move(robot_id)), node_(node), model_(std::move(model)) {
  ros::NodeHandle config(*node_, "skid_steer");
  config.param("wheel_separation", track_, 0.416503);
  config.param("wheel_radius", radius_, 0.08);
  config.param("command_gain", gain_, 1.01);
  config.param("angular_command_gain", yaw_gain_, 1.46);
  config.param("wheel_pid_p", p_, 1.8);
  config.param("wheel_pid_i", i_, 10.0);
  config.param("max_linear_speed", max_v_, 1.5);
  config.param("max_angular_speed", max_w_, 1.0);
  config.param("enable_command_limits", limits_, true);
  double delay;
  config.param("command_delay_s", delay, 0.005);
  if (!std::isfinite(delay) || delay < 0) throw std::invalid_argument("Scout command delay must be finite and nonnegative");
  for (double value : {track_, radius_, max_v_, max_w_, i_})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Scout geometry, limits and I must be positive");
  if (!std::isfinite(gain_) || !std::isfinite(yaw_gain_) || !std::isfinite(p_) || p_ < 0)
    throw std::invalid_argument("Scout command gains and P must be finite; P must be nonnegative");
  delay_.Configure(delay);
  for (std::size_t index=0; index<4; ++index) {
    joints_[index] = model_->GetJoint(names_[index]);
    if (!joints_[index]) throw std::runtime_error("Scout native skid requires all four wheel joints");
    effort_limit_[index] = joints_[index]->GetEffortLimit(0);
    velocity_limit_[index] = joints_[index]->GetVelocityLimit(0);
    if (!(effort_limit_[index] > 0 && velocity_limit_[index] > 0))
      throw std::runtime_error("Scout native skid requires positive URDF wheel limits");
  }
  time_ = model_->GetWorld()->SimTime().Double();
}
ScoutSkidSteer::~ScoutSkidSteer() {
  update_.reset();
  command_.shutdown();
  if (callbacks_) callbacks_->drain();
  // Native RetireEntity already zeroed before Gazebo Fini; unregister only drains.
  binding_.reset();
}
void ScoutSkidSteer::SetupSubscription() {
  joint_states_ = node_->advertise<sensor_msgs::JointState>("joint_states", 1);
  callbacks_.reset(new xgc2_gazebo_scene::ChassisRosCallbacks(model_->GetWorld()));
  node_->setCallbackQueue(callbacks_->queue());
  binding_.reset(new xgc2_gazebo_scene::ChassisBinding(
      model_->GetWorld(), robot_id_, &ScoutSkidSteer::HoldZeroThunk, this));
  command_ = node_->subscribe<geometry_msgs::Twist>("cmd_vel", 5,
      callbacks_->wrap([this](const geometry_msgs::Twist::ConstPtr& command) { Command(command); }));
  update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      callbacks_->wrap([this](const gazebo::common::UpdateInfo& info) { Update(info); }));
  callbacks_->invalidate();
  binding_->Ready();
}
void ScoutSkidSteer::HoldZeroThunk(void* context) { static_cast<ScoutSkidSteer*>(context)->Zero(); }
void ScoutSkidSteer::Zero() {
  if (callbacks_) callbacks_->invalidate();
  delay_.Reset();
  target_.fill(0.0);
  state_ = {};
  for (const auto& joint : joints_) if (joint) joint->SetForce(0, 0.0);
}
void ScoutSkidSteer::Reset() {
  if (!binding_) return;
  binding_->with_command([this](bool) {
    Zero();
    time_ = model_->GetWorld()->SimTime().Double();
    publish_time_ = 0;
  });
}
void ScoutSkidSteer::Command(const geometry_msgs::Twist::ConstPtr& message) {
  if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) return;
  binding_->with_command([this, &message](bool held) {
    if (held || !callbacks_->current()) return;
    const double v = limits_ ? std::clamp(message->linear.x, -max_v_, max_v_) : message->linear.x;
    const double w = limits_ ? std::clamp(message->angular.z, -max_w_, max_w_) : message->angular.z;
    delay_.Push(time_, v, w);
  });
}
void ScoutSkidSteer::Update(const gazebo::common::UpdateInfo& info) {
  binding_->with_command([this, &info](bool held) {
    const double now = info.simTime.Double();
    const double dt = now >= time_ ? now-time_ : 0;
    if (now < time_) { Zero(); publish_time_ = 0; }
    time_ = now;
    if (held) { Zero(); } else {
    const auto command = delay_.Advance(now);
    const auto wheels = xgc2_math::differentialDriveWheelVelocity(
        command.linear, command.angular, {radius_, track_, gain_, yaw_gain_});
    target_ = {{wheels.right_rad_s, wheels.left_rad_s, wheels.left_rad_s, wheels.right_rad_s}};
    for (std::size_t index=0; index<4; ++index) {
      target_[index] = std::clamp(target_[index], -velocity_limit_[index], velocity_limit_[index]);
      if (dt > 0) state_[index] = xgc2_math::wheelVelocityIPStep(state_[index], target_[index],
          joints_[index]->GetVelocity(0), dt, {p_, i_, effort_limit_[index]});
      joints_[index]->SetForce(0, state_[index].effort_nm);
    }
    }
    if (now >= publish_time_) {
      sensor_msgs::JointState message;
      message.header.stamp = ros::Time(now);
      message.name.assign(names_.begin(), names_.end());
      for (std::size_t index=0; index<4; ++index) {
        message.position.push_back(joints_[index]->Position(0));
        message.velocity.push_back(joints_[index]->GetVelocity(0));
        message.effort.push_back(state_[index].effort_nm);
      }
      joint_states_.publish(message);
      publish_time_ = now + 0.01;
    }
  });
}
}
