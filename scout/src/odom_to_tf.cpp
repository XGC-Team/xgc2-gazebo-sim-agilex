#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

class OdomToTf {
public:
  explicit OdomToTf(ros::NodeHandle& private_nh) {
    private_nh.param<std::string>("odom_topic", odom_topic_, "odom");
    odom_sub_ = nh_.subscribe(odom_topic_, 20, &OdomToTf::odomCallback, this);
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (msg->header.frame_id.empty() || msg->child_frame_id.empty()) {
      ROS_WARN_THROTTLE(5.0, "Ignoring odometry without frame ids");
      return;
    }
    if (!last_stamp_.isZero() && msg->header.stamp <= last_stamp_) {
      return;
    }
    last_stamp_ = msg->header.stamp;

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header = msg->header;
    tf_msg.child_frame_id = msg->child_frame_id;
    tf_msg.transform.translation.x = msg->pose.pose.position.x;
    tf_msg.transform.translation.y = msg->pose.pose.position.y;
    tf_msg.transform.translation.z = msg->pose.pose.position.z;
    tf_msg.transform.rotation = msg->pose.pose.orientation;
    broadcaster_.sendTransform(tf_msg);
  }

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  tf2_ros::TransformBroadcaster broadcaster_;
  std::string odom_topic_;
  ros::Time last_stamp_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_to_tf");
  ros::NodeHandle private_nh("~");
  OdomToTf node(private_nh);
  ros::spin();
  return 0;
}
