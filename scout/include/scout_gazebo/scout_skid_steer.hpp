/*
 * scout_skid_steer.hpp
 *
 * Created on: Mar 25, 2020 22:52
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_SKID_STEER_HPP
#define SCOUT_SKID_STEER_HPP

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <string>

#include "xgc_chassis_hold/udp.hpp"
#include "scout_gazebo/command_dynamics.hpp"

namespace wescore {
class ScoutSkidSteer {
public:
  ScoutSkidSteer(ros::NodeHandle *nh, std::string robot_name = "");
  ~ScoutSkidSteer();

  void SetupSubscription();

private:
  std::string robot_name_;
  std::string motor_fr_topic_;
  std::string motor_fl_topic_;
  std::string motor_rl_topic_;
  std::string motor_rr_topic_;
  std::string cmd_topic_;

  double wheel_separation_;
  double wheel_radius_;
  double command_gain_;
  double angular_command_gain_;
  double command_delay_s_;
  double command_time_constant_s_;
  bool enable_command_limits_;
  double max_linear_speed_;
  double max_angular_speed_;

  CommandDynamics command_dynamics_;

  ros::NodeHandle *nh_;

  ros::Publisher motor_fr_pub_;
  ros::Publisher motor_fl_pub_;
  ros::Publisher motor_rl_pub_;
  ros::Publisher motor_rr_pub_;

  ros::Subscriber cmd_sub_;
  ros::WallTimer control_timer_;
  xgc_chassis_hold::Gate hold_gate_;

  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void PublishZeroMotors();
  static void HoldZeroThunk(void *self);
  void ControlTick(const ros::WallTimerEvent &event);
  double Clamp(double value, double limit) const;
  std::string JoinTopic(const std::string &ns, const std::string &topic) const;
};
} // namespace wescore

#endif /* SCOUT_SKID_STEER_HPP */
