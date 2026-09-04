#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>

// Plant ground truth for simulation readiness. Canonical /<ns>/pose and
// /twist belong to the Robot Adapter. Do not advertise chassis odometry.
class GazeboModelGroundTruth {
public:
  explicit GazeboModelGroundTruth(ros::NodeHandle &private_nh) {
    private_nh.param<std::string>("model_name", model_name_, "scout");
    private_nh.param<std::string>("world_frame", world_frame_, "world");
    private_nh.param<std::string>("base_frame", base_frame_, "base_link");
    private_nh.param<std::string>("model_states_topic", model_states_topic_,
                                  "/gazebo/model_states");
    private_nh.param<std::string>("pose_topic", pose_topic_,
                                  "simulation/ground_truth/pose");
    private_nh.param<std::string>("twist_topic", twist_topic_,
                                  "simulation/ground_truth/twist");
    private_nh.param("publish_tf", publish_tf_, true);

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
    twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(twist_topic_, 10);
    model_states_sub_ =
        nh_.subscribe(model_states_topic_, 10,
                      &GazeboModelGroundTruth::modelStatesCallback, this);
  }

private:
  void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] != model_name_) {
        continue;
      }

      const ros::Time stamp = ros::Time::now();
      if (!last_stamp_.isZero() && stamp <= last_stamp_) {
        return;
      }
      last_stamp_ = stamp;

      geometry_msgs::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = world_frame_;
      pose.pose = msg->pose[i];
      pose_pub_.publish(pose);

      geometry_msgs::TwistStamped twist;
      twist.header = pose.header;
      twist.twist = msg->twist[i];
      twist_pub_.publish(twist);

      if (publish_tf_) {
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header = pose.header;
        tf_msg.child_frame_id = base_frame_;
        tf_msg.transform.translation.x = msg->pose[i].position.x;
        tf_msg.transform.translation.y = msg->pose[i].position.y;
        tf_msg.transform.translation.z = msg->pose[i].position.z;
        tf_msg.transform.rotation = msg->pose[i].orientation;
        broadcaster_.sendTransform(tf_msg);
      }
      return;
    }

    ROS_WARN_THROTTLE(5.0, "Gazebo model '%s' was not found in %s",
                      model_name_.c_str(), model_states_topic_.c_str());
  }

  ros::NodeHandle nh_;
  ros::Subscriber model_states_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher twist_pub_;
  tf2_ros::TransformBroadcaster broadcaster_;
  std::string model_name_;
  std::string world_frame_;
  std::string base_frame_;
  std::string model_states_topic_;
  std::string pose_topic_;
  std::string twist_topic_;
  ros::Time last_stamp_;
  bool publish_tf_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "gazebo_model_ground_truth");
  ros::NodeHandle private_nh("~");
  GazeboModelGroundTruth node(private_nh);
  ros::spin();
  return 0;
}
