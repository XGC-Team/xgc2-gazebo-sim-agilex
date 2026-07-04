/*
 * scout_skid_steer.cpp
 *
 * Created on: Mar 25, 2020 22:54
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_gazebo/scout_skid_steer.hpp"

#include <cmath>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

namespace wescore {
ScoutSkidSteer::ScoutSkidSteer(ros::NodeHandle *nh, std::string robot_name)
    : nh_(nh), robot_name_(robot_name) {
  ros::NodeHandle private_nh("~");
  private_nh.param("wheel_separation", wheel_separation_, 0.416503);
  private_nh.param("wheel_radius", wheel_radius_, 0.08);
  private_nh.param("command_gain", command_gain_, 1.0);
  private_nh.param("angular_command_gain", angular_command_gain_, 1.0);

  motor_fr_topic_ = JoinTopic(robot_name_, "scout_motor_fr_controller/command");
  motor_fl_topic_ = JoinTopic(robot_name_, "scout_motor_fl_controller/command");
  motor_rl_topic_ = JoinTopic(robot_name_, "scout_motor_rl_controller/command");
  motor_rr_topic_ = JoinTopic(robot_name_, "scout_motor_rr_controller/command");
  cmd_topic_ = JoinTopic(robot_name_, "cmd_vel");

  ROS_INFO("Scout skid steer: cmd=%s fr=%s fl=%s rl=%s rr=%s wheel_separation=%.6f wheel_radius=%.6f gain=%.3f angular_gain=%.3f",
           cmd_topic_.c_str(), motor_fr_topic_.c_str(), motor_fl_topic_.c_str(),
           motor_rl_topic_.c_str(), motor_rr_topic_.c_str(), wheel_separation_,
           wheel_radius_, command_gain_, angular_command_gain_);
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
  double steering_vel = msg->angular.z * angular_command_gain_;
  if (!std::isfinite(driving_vel) || !std::isfinite(steering_vel) ||
      wheel_radius_ <= 0.0) {
    ROS_WARN_THROTTLE(1.0, "Ignoring invalid Scout cmd_vel or wheel radius");
    return;
  }
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

}  // namespace wescore
