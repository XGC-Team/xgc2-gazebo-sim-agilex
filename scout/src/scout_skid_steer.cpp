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

namespace wescore {
ScoutSkidSteer::ScoutSkidSteer(ros::NodeHandle *nh, std::string robot_name)
    : robot_name_(robot_name), command_delay_s_(0.15),
      command_time_constant_s_(0.15), delayed_linear_velocity_(0.0),
      delayed_angular_velocity_(0.0), filtered_linear_velocity_(0.0),
      filtered_angular_velocity_(0.0), nh_(nh) {
  ros::NodeHandle private_nh("~");
  private_nh.param("wheel_separation", wheel_separation_, 0.490);
  private_nh.param("wheel_radius", wheel_radius_, 0.08);
  private_nh.param("command_gain", command_gain_, 1.0);
  private_nh.param("angular_command_gain", angular_command_gain_, 1.0);
  private_nh.param("command_delay_s", command_delay_s_, 0.15);
  private_nh.param("command_time_constant_s", command_time_constant_s_, 0.15);
  private_nh.param("enable_command_limits", enable_command_limits_, true);
  private_nh.param("max_linear_speed", max_linear_speed_, 1.5);
  private_nh.param("max_angular_speed", max_angular_speed_, 0.5235);

  if (!std::isfinite(command_delay_s_) || command_delay_s_ < 0.0) {
    ROS_WARN("Invalid Scout command_delay_s %.6f; using 0", command_delay_s_);
    command_delay_s_ = 0.0;
  }
  if (!std::isfinite(command_time_constant_s_) ||
      command_time_constant_s_ < 0.0) {
    ROS_WARN("Invalid Scout command_time_constant_s %.6f; using 0",
             command_time_constant_s_);
    command_time_constant_s_ = 0.0;
  }

  motor_fr_topic_ = JoinTopic(robot_name_, "scout_motor_fr_controller/command");
  motor_fl_topic_ = JoinTopic(robot_name_, "scout_motor_fl_controller/command");
  motor_rl_topic_ = JoinTopic(robot_name_, "scout_motor_rl_controller/command");
  motor_rr_topic_ = JoinTopic(robot_name_, "scout_motor_rr_controller/command");
  cmd_topic_ = JoinTopic(robot_name_, "cmd_vel");

  ROS_INFO(
      "Scout skid steer: cmd=%s fr=%s fl=%s rl=%s rr=%s wheel_separation=%.6f "
      "wheel_radius=%.6f gain=%.3f angular_gain=%.3f command_delay=%.3f "
      "command_tau=%.3f limits=%s max_linear=%.4f max_angular=%.4f",
      cmd_topic_.c_str(), motor_fr_topic_.c_str(), motor_fl_topic_.c_str(),
      motor_rl_topic_.c_str(), motor_rr_topic_.c_str(), wheel_separation_,
      wheel_radius_, command_gain_, angular_command_gain_, command_delay_s_,
      command_time_constant_s_, enable_command_limits_ ? "true" : "false",
      max_linear_speed_, max_angular_speed_);
}

void ScoutSkidSteer::SetupSubscription() {
  // command subscriber
  cmd_sub_ = nh_->subscribe<geometry_msgs::Twist>(
      cmd_topic_, 5, &ScoutSkidSteer::TwistCmdCallback, this);

  // motor command publisher
  motor_fr_pub_ = nh_->advertise<std_msgs::Float64>(motor_fr_topic_, 50);
  motor_fl_pub_ = nh_->advertise<std_msgs::Float64>(motor_fl_topic_, 50);
  motor_rl_pub_ = nh_->advertise<std_msgs::Float64>(motor_rl_topic_, 50);
  motor_rr_pub_ = nh_->advertise<std_msgs::Float64>(motor_rr_topic_, 50);
}

void ScoutSkidSteer::TwistCmdCallback(
    const geometry_msgs::Twist::ConstPtr &msg) {
  double driving_vel = msg->linear.x;
  double steering_vel = msg->angular.z;
  if (!std::isfinite(driving_vel) || !std::isfinite(steering_vel) ||
      wheel_radius_ <= 0.0) {
    ROS_WARN_THROTTLE(1.0, "Ignoring invalid Scout cmd_vel or wheel radius");
    return;
  }
  if (enable_command_limits_) {
    double limited_driving_vel = Clamp(driving_vel, max_linear_speed_);
    double limited_steering_cmd = Clamp(steering_vel, max_angular_speed_);
    if (limited_driving_vel != driving_vel ||
        limited_steering_cmd != steering_vel) {
      ROS_WARN_THROTTLE(1.0,
                        "Scout cmd_vel limited: linear %.4f -> %.4f, angular "
                        "%.4f -> %.4f",
                        driving_vel, limited_driving_vel, steering_vel,
                        limited_steering_cmd);
    }
    driving_vel = limited_driving_vel;
    steering_vel = limited_steering_cmd;
  }
  UpdateCommandDynamics(driving_vel, steering_vel, ros::Time::now(),
                        &driving_vel, &steering_vel);
  steering_vel *= angular_command_gain_;
  const double half_track = wheel_separation_ * 0.5;

  double left_side_velocity =
      (driving_vel - steering_vel * half_track) / wheel_radius_;
  double right_side_velocity =
      (driving_vel + steering_vel * half_track) / wheel_radius_;

  std_msgs::Float64 motor_cmd[4];
  motor_cmd[0].data = right_side_velocity * command_gain_;
  motor_cmd[1].data = left_side_velocity * command_gain_;
  motor_cmd[2].data = left_side_velocity * command_gain_;
  motor_cmd[3].data = right_side_velocity * command_gain_;

  motor_fr_pub_.publish(motor_cmd[0]);
  motor_fl_pub_.publish(motor_cmd[1]);
  motor_rl_pub_.publish(motor_cmd[2]);
  motor_rr_pub_.publish(motor_cmd[3]);
}

void ScoutSkidSteer::UpdateCommandDynamics(double requested_linear,
                                           double requested_angular,
                                           const ros::Time &now, double *linear,
                                           double *angular) {
  if (last_update_time_.isZero() || now < last_update_time_) {
    command_history_.clear();
    delayed_linear_velocity_ = 0.0;
    delayed_angular_velocity_ = 0.0;
    filtered_linear_velocity_ = 0.0;
    filtered_angular_velocity_ = 0.0;
    last_update_time_ = now;
  }

  command_history_.push_back({now, requested_linear, requested_angular});
  if (now.toSec() >= command_delay_s_) {
    const ros::Time cutoff = now - ros::Duration(command_delay_s_);
    while (!command_history_.empty() &&
           command_history_.front().stamp <= cutoff) {
      delayed_linear_velocity_ = command_history_.front().linear;
      delayed_angular_velocity_ = command_history_.front().angular;
      command_history_.pop_front();
    }
  }
  while (command_history_.size() > 2048U) {
    command_history_.pop_front();
    ROS_WARN_THROTTLE(1.0, "Scout command delay queue exceeded 2048 samples");
  }

  const double dt = std::max(0.0, (now - last_update_time_).toSec());
  double alpha = 1.0;
  if (command_time_constant_s_ > 0.0) {
    alpha = 1.0 - std::exp(-dt / command_time_constant_s_);
  }
  filtered_linear_velocity_ +=
      alpha * (delayed_linear_velocity_ - filtered_linear_velocity_);
  filtered_angular_velocity_ +=
      alpha * (delayed_angular_velocity_ - filtered_angular_velocity_);
  last_update_time_ = now;

  *linear = filtered_linear_velocity_;
  *angular = filtered_angular_velocity_;
}

double ScoutSkidSteer::Clamp(double value, double limit) const {
  if (limit <= 0.0) {
    return value;
  }
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

std::string ScoutSkidSteer::JoinTopic(const std::string &ns,
                                      const std::string &topic) const {
  if (ns.empty() || ns == "/") {
    return "/" + topic;
  }

  std::string normalized = ns;
  if (normalized.front() != '/') {
    normalized = "/" + normalized;
  }
  while (!normalized.empty() && normalized.back() == '/') {
    normalized.pop_back();
  }
  return normalized + "/" + topic;
}

} // namespace wescore
