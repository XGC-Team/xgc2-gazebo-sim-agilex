#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <string>

#include "scout_gazebo/scout_skid_steer.hpp"

using namespace wescore;

int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "scout_skid_steer_controller");
  ros::NodeHandle node(""), private_node("~");

  // fetch parameters
  std::string ns;
  private_node.param<std::string>("ns", ns, std::string("scout_default"));

  ROS_INFO("Namespace: %s", ns.c_str());

  ScoutSkidSteer skid_steer_controller(&node, ns);
  skid_steer_controller.SetupSubscription();

  ros::spin();

  return 0;
}
