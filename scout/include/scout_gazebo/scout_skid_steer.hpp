#pragma once
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <xgc2_math/control/wheel_drive.hpp>
#include <xgc2_gazebo_scene/chassis_hold.hpp>
#include "scout_gazebo/command_delay.hpp"
#include <array>
#include <memory>
#include <string>

namespace wescore {
// Same delayed differential allocation and wheel I/P law, now owned by the
// native Gazebo model. No intermediate ROS motor-control pipeline exists.
class ScoutSkidSteer {
 public:
  ScoutSkidSteer(ros::NodeHandle* node, gazebo::physics::ModelPtr model, std::string robot_id);
  ~ScoutSkidSteer();
  void SetupSubscription();
  void Update(const gazebo::common::UpdateInfo& info);
  void Reset();
 private:
  void Zero();
  void Command(const geometry_msgs::Twist::ConstPtr& command);
  std::string robot_id_;
  ros::NodeHandle* node_;
  gazebo::physics::ModelPtr model_;
  std::array<gazebo::physics::JointPtr, 4> joints_;
  std::array<xgc2_math::WheelVelocityIPState, 4> state_{};
  std::array<double, 4> target_{};
  std::array<double, 4> effort_limit_{}, velocity_limit_{};
  std::array<std::string, 4> names_{{"front_right_wheel", "front_left_wheel", "rear_left_wheel", "rear_right_wheel"}};
  double track_{}, radius_{}, gain_{}, yaw_gain_{}, p_{}, i_{};
  double max_v_{}, max_w_{}, time_{}, publish_time_{};
  bool limits_ = true;
  CommandDelay delay_;
  ros::Publisher joint_states_;
  ros::Subscriber command_;
  gazebo::event::ConnectionPtr update_;
  std::unique_ptr<xgc2_gazebo_scene::ChassisHold> hold_;
};
}
