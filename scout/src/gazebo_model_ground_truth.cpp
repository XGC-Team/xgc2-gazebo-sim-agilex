#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

#include <cmath>
#include <string>

// Plant ground truth for simulation readiness. Canonical /<ns>/pose and
// /twist belong to the Robot Adapter. Do not advertise chassis odometry.
class GazeboModelGroundTruth {
public:
  explicit GazeboModelGroundTruth(ros::NodeHandle &private_nh) {
    private_nh.param<std::string>("model_name", model_name_, "scout");
    private_nh.param<std::string>("world_frame", world_frame_, "world");
    private_nh.param<std::string>("model_states_topic", model_states_topic_,
                                  "/gazebo/model_states");
    private_nh.param<std::string>("pose_topic", pose_topic_,
                                  "simulation/ground_truth/pose");
    private_nh.param<std::string>("twist_topic", twist_topic_,
                                  "simulation/ground_truth/twist");

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
    twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(twist_topic_, 10);
    model_states_sub_ =
        nh_.subscribe(model_states_topic_, 10,
                      &GazeboModelGroundTruth::modelStatesCallback, this);
  }

private:
  static bool validSample(const geometry_msgs::Pose &pose,
                          const geometry_msgs::Twist &twist) {
    const double values[] = {
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z,
        pose.orientation.w, twist.linear.x, twist.linear.y, twist.linear.z,
        twist.angular.x, twist.angular.y, twist.angular.z};
    for (double value : values) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
    const auto &q = pose.orientation;
    const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(norm_squared) && norm_squared > 0.0;
  }

  void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
    // ModelStates has no source stamp. Use one reception-time ROS stamp for
    // both outputs, never wall time or a fabricated monotonic timestamp.
    const ros::Time stamp = ros::Time::now();
    if (stamp.isZero() ||
        (!last_observed_stamp_.isZero() && stamp < last_observed_stamp_)) {
      // A new simulation epoch must not wait to catch up with the old run.
      // Observe this even while the selected model is absent or malformed.
      last_stamp_ = ros::Time();
    }
    last_observed_stamp_ = stamp;
    if (stamp.isZero()) {
      // No valid simulation clock yet, or reset_simulation has returned to 0.
      return;
    }
    if (!last_stamp_.isZero() && stamp == last_stamp_) {
      return;
    }
    if (msg->name.size() != msg->pose.size() ||
        msg->name.size() != msg->twist.size()) {
      ROS_WARN_THROTTLE(5.0, "Ignoring malformed Gazebo ModelStates arrays");
      return;
    }

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] != model_name_) {
        continue;
      }

      // Validate the complete selected sample before publishing either half.
      // Rejected input must not consume a stamp needed by a later valid sample.
      if (!validSample(msg->pose[i], msg->twist[i])) {
        ROS_WARN_THROTTLE(5.0, "Ignoring invalid Gazebo state for model '%s'",
                          model_name_.c_str());
        return;
      }

      geometry_msgs::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = world_frame_;
      pose.pose = msg->pose[i];
      pose_pub_.publish(pose);

      geometry_msgs::TwistStamped twist;
      twist.header = pose.header;
      twist.twist = msg->twist[i];
      twist_pub_.publish(twist);
      last_stamp_ = stamp;

      return;
    }

    ROS_WARN_THROTTLE(5.0, "Gazebo model '%s' was not found in %s",
                      model_name_.c_str(), model_states_topic_.c_str());
  }

  ros::NodeHandle nh_;
  ros::Subscriber model_states_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher twist_pub_;
  std::string model_name_;
  std::string world_frame_;
  std::string model_states_topic_;
  std::string pose_topic_;
  std::string twist_topic_;
  ros::Time last_stamp_;
  ros::Time last_observed_stamp_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "gazebo_model_ground_truth");
  ros::NodeHandle private_nh("~");
  GazeboModelGroundTruth node(private_nh);
  ros::spin();
  return 0;
}
