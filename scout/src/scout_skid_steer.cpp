/*
 * scout_skid_steer.cpp
 *
 * Created on: Mar 25, 2020 22:54
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_gazebo/scout_skid_steer.hpp"

#include <algorithm>
#include <cmath>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

namespace wescore {
ScoutSkidSteer::ScoutSkidSteer(ros::NodeHandle *nh, std::string robot_name)
    : robot_name_(robot_name), command_delay_s_(0.005),
      command_dispatch_period_s_(0.001), nh_(nh),
      hold_gate_(xgc_chassis_hold::lastPath(robot_name)) {
  ros::NodeHandle private_nh("~");
  private_nh.param("wheel_separation", wheel_separation_, 0.416503);
  private_nh.param("wheel_radius", wheel_radius_, 0.08);
  private_nh.param("command_gain", command_gain_, 1.0);
  private_nh.param("angular_command_gain", angular_command_gain_, 1.0);
  private_nh.param("command_delay_s", command_delay_s_, 0.005);
  private_nh.param("command_dispatch_period_s", command_dispatch_period_s_, 0.001);
  private_nh.param("enable_command_limits", enable_command_limits_, true);
  private_nh.param("max_linear_speed", max_linear_speed_, 1.5);
  private_nh.param("max_angular_speed", max_angular_speed_, 0.5235);

  if (!std::isfinite(command_delay_s_) || command_delay_s_ < 0.0 ||
      !std::isfinite(command_dispatch_period_s_) || command_dispatch_period_s_ <= 0.0) {
    throw std::invalid_argument("Scout delay must be nonnegative and dispatch period positive");
  }
  if (!std::isfinite(wheel_separation_) || wheel_separation_ <= 0.0 ||
      !std::isfinite(angular_command_gain_) || angular_command_gain_ <= 0.0 ||
      !std::isfinite(max_linear_speed_) || !std::isfinite(max_angular_speed_) ||
      (enable_command_limits_ && (max_linear_speed_ <= 0.0 || max_angular_speed_ <= 0.0))) {
    throw std::invalid_argument("Scout geometry, angular gain and enabled command limits must be positive and finite");
  }
  wheel_allocation_.radius_m = wheel_radius_;
  wheel_allocation_.common_gain = command_gain_;
  wheel_allocation_.allocation_span_m = wheel_separation_ * angular_command_gain_;
  // An explicit command-map span never moves contact geometry. Legacy launch
  // arguments retain precisely their old product when this parameter is absent.
  if (private_nh.hasParam("allocation_span_m")) {
    if (!private_nh.getParam("allocation_span_m", wheel_allocation_.allocation_span_m)) {
      throw std::invalid_argument("Scout allocation_span_m must be numeric");
    }
    ROS_INFO("Scout explicit allocation_span_m supersedes wheel_separation * angular_command_gain for commands only");
  }
  wheel_allocation_.Validate();
  command_delay_.Configure(command_delay_s_);

  motor_fr_topic_ = JoinTopic(robot_name_, "scout_motor_fr_controller/command");
  motor_fl_topic_ = JoinTopic(robot_name_, "scout_motor_fl_controller/command");
  motor_rl_topic_ = JoinTopic(robot_name_, "scout_motor_rl_controller/command");
  motor_rr_topic_ = JoinTopic(robot_name_, "scout_motor_rr_controller/command");
  cmd_topic_ = JoinTopic(robot_name_, "cmd_vel");

  ROS_INFO(
      "Scout skid steer: cmd=%s fr=%s fl=%s rl=%s rr=%s wheel_separation=%.6f "
      "wheel_radius=%.6f gain=%.3f angular_gain=%.3f allocation_span_m=%.9f "
      "command_delay=%.6f dispatch_period_sim_s=%.6f added_input_lag_s=0 "
      "limits=%s max_linear=%.4f max_angular=%.4f",
      cmd_topic_.c_str(), motor_fr_topic_.c_str(), motor_fl_topic_.c_str(),
      motor_rl_topic_.c_str(), motor_rr_topic_.c_str(), wheel_separation_,
      wheel_radius_, command_gain_, angular_command_gain_,
      wheel_allocation_.allocation_span_m, command_delay_s_, command_dispatch_period_s_,
      enable_command_limits_ ? "true" : "false", max_linear_speed_, max_angular_speed_);
}

ScoutSkidSteer::~ScoutSkidSteer() {
  control_timer_.stop();
  cmd_sub_.shutdown();
  // remove() drains any UDP callback before the Gate and publishers die.
  xgc_chassis_hold::Hub::instance().remove(&hold_gate_);
}

void ScoutSkidSteer::SetupSubscription() {
  motor_fr_pub_ = nh_->advertise<std_msgs::Float64>(motor_fr_topic_, 50);
  motor_fl_pub_ = nh_->advertise<std_msgs::Float64>(motor_fl_topic_, 50);
  motor_rl_pub_ = nh_->advertise<std_msgs::Float64>(motor_rl_topic_, 50);
  motor_rr_pub_ = nh_->advertise<std_msgs::Float64>(motor_rr_topic_, 50);
  dispatch_trace_pub_ = nh_->advertise<std_msgs::Float64MultiArray>(
      JoinTopic(robot_name_, "command_dispatch_trace"), 20);
  hold_gate_.setZeroThunk(&ScoutSkidSteer::HoldZeroThunk, this);
  xgc_chassis_hold::Hub::instance().add(&hold_gate_);
  cmd_sub_ = nh_->subscribe<geometry_msgs::Twist>(
      cmd_topic_, 5, &ScoutSkidSteer::TwistCmdCallback, this);
  // Both queue due times and polling now use simulation time. A 10 ms WALL
  // timer makes the extra dispatch error depend on real-time factor and load.
  // Physics/ROS callback quantization still exists and is measured below; the
  // 5 ms transport parameter is not a claim of 5 ms command-to-torque latency.
  // Emergency hold remains an independent UDP callback and flushes the queue.
  control_timer_ = nh_->createTimer(
      ros::Duration(command_dispatch_period_s_), &ScoutSkidSteer::ControlTick, this);
}

void ScoutSkidSteer::HoldZeroThunk(void *self) {
  static_cast<ScoutSkidSteer *>(self)->PublishZeroMotors();
}

void ScoutSkidSteer::PublishZeroMotors() {
  command_delay_.Reset();
  if (!motor_fr_pub_) return;
  std_msgs::Float64 motor_cmd[4];
  motor_fr_pub_.publish(motor_cmd[0]);
  motor_fl_pub_.publish(motor_cmd[1]);
  motor_rl_pub_.publish(motor_cmd[2]);
  motor_rr_pub_.publish(motor_cmd[3]);
}

void ScoutSkidSteer::TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  double driving_vel = msg->linear.x;
  double steering_vel = msg->angular.z;
  if (!std::isfinite(driving_vel) || !std::isfinite(steering_vel)) {
    ROS_WARN_THROTTLE(1.0, "Ignoring invalid Scout cmd_vel");
    return;
  }
  if (enable_command_limits_) {
    const double limited_driving_vel = Clamp(driving_vel, max_linear_speed_);
    const double limited_steering_cmd = Clamp(steering_vel, max_angular_speed_);
    if (limited_driving_vel != driving_vel || limited_steering_cmd != steering_vel) {
      ROS_WARN_THROTTLE(1.0, "Scout cmd_vel limited: linear %.4f -> %.4f, angular %.4f -> %.4f",
                       driving_vel, limited_driving_vel, steering_vel, limited_steering_cmd);
    }
    driving_vel = limited_driving_vel;
    steering_vel = limited_steering_cmd;
  }
  hold_gate_.withCommand([&](bool held) {
    if (held) {
      PublishZeroMotors();
      return;
    }
    command_delay_.Push(ros::Time::now().toSec(), driving_vel, steering_vel);
  });
}

void ScoutSkidSteer::ControlTick(const ros::TimerEvent &event) {
  hold_gate_.withCommand([this, &event](bool held) {
    if (held) {
      PublishZeroMotors();
      return;
    }
    const double dispatch_s = ros::Time::now().toSec();
    const CommandVelocity command = command_delay_.Advance(dispatch_s);
    const auto wheels = wheel_allocation_.Apply(command.linear, command.angular);
    std_msgs::Float64 motor_cmd[4];
    for (int i = 0; i < 4; ++i) motor_cmd[i].data = wheels[i];
    motor_fr_pub_.publish(motor_cmd[0]);
    motor_fl_pub_.publish(motor_cmd[1]);
    motor_rl_pub_.publish(motor_cmd[2]);
    motor_rr_pub_.publish(motor_cmd[3]);
    if (dispatch_trace_pub_.getNumSubscribers() > 0 && command_delay_.Sample().valid) {
      const auto &sample = command_delay_.Sample();
      std_msgs::Float64MultiArray trace;
      trace.layout.dim.resize(1);
      trace.layout.dim[0].label = "v1:seq,receive_s,due_s,dispatch_s,timer_expected_s,v_mps,w_radps,fr_radps,fl_radps,rl_radps,rr_radps";
      trace.layout.dim[0].size = 11;
      trace.layout.dim[0].stride = 11;
      trace.data = {static_cast<double>(sample.sequence), sample.received_s, sample.due_s,
                    dispatch_s, event.current_expected.toSec(), command.linear, command.angular,
                    wheels[0], wheels[1], wheels[2], wheels[3]};
      dispatch_trace_pub_.publish(trace);
    }
  });
}

double ScoutSkidSteer::Clamp(double value, double limit) const {
  if (limit <= 0.0) return value;
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

std::string ScoutSkidSteer::JoinTopic(const std::string &ns, const std::string &topic) const {
  if (ns.empty() || ns == "/") return "/" + topic;
  std::string normalized = ns;
  if (normalized.front() != '/') normalized = "/" + normalized;
  while (!normalized.empty() && normalized.back() == '/') normalized.pop_back();
  return normalized + "/" + topic;
}

} // namespace wescore
