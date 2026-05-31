#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>

class GazeboModelOdom {
 public:
  explicit GazeboModelOdom(ros::NodeHandle& private_nh) {
    private_nh.param<std::string>("model_name", model_name_, "scout");
    private_nh.param<std::string>("odom_frame", odom_frame_, "odom");
    private_nh.param<std::string>("base_frame", base_frame_, "base_link");
    private_nh.param<std::string>("model_states_topic", model_states_topic_,
                                  "/gazebo/model_states");
    private_nh.param<std::string>("odom_topic", odom_topic_, "odom");
    private_nh.param("publish_tf", publish_tf_, true);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    model_states_sub_ = nh_.subscribe(model_states_topic_, 10,
                                      &GazeboModelOdom::modelStatesCallback,
                                      this);
  }

 private:
  void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] != model_name_) {
        continue;
      }

      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now();
      if (!last_stamp_.isZero() && odom.header.stamp <= last_stamp_) {
        return;
      }
      last_stamp_ = odom.header.stamp;
      odom.header.frame_id = odom_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose = msg->pose[i];
      odom.twist.twist = msg->twist[i];
      odom_pub_.publish(odom);

      if (publish_tf_) {
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header = odom.header;
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
  ros::Publisher odom_pub_;
  tf2_ros::TransformBroadcaster broadcaster_;
  std::string model_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string model_states_topic_;
  std::string odom_topic_;
  ros::Time last_stamp_;
  bool publish_tf_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gazebo_model_odom");
  ros::NodeHandle private_nh("~");
  GazeboModelOdom node(private_nh);
  ros::spin();
  return 0;
}
