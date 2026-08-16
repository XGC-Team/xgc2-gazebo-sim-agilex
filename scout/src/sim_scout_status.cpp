#include <cmath>
#include <cstdio>
#include <map>
#include <stdint.h>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <scout_msgs/ScoutLightCmd.h>
#include <scout_msgs/ScoutStatus.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

class SimScoutStatus {
 public:
  SimScoutStatus() : private_nh_("~"), have_odom_(false) {
    private_nh_.param<std::string>("odom_topic", odom_topic_, "odom");
    private_nh_.param<std::string>("joint_states_topic", joint_states_topic_,
                                   "joint_states");
    private_nh_.param<std::string>("status_topic", status_topic_,
                                   "scout_status");
    private_nh_.param<std::string>("status_text_topic", status_text_topic_,
                                   "");
    if (status_text_topic_.empty()) {
      status_text_topic_ = DeriveStatusTextTopic(status_topic_);
    }
    private_nh_.param<std::string>("light_control_topic", light_control_topic_,
                                   "scout_light_control");
    private_nh_.param("publish_rate", publish_rate_, 50.0);
    private_nh_.param("base_state", base_state_, 0);
    private_nh_.param("control_mode", control_mode_, 1);
    private_nh_.param("fault_code", fault_code_, 0);
    // Fixed 95% Scout pack voltage using the manual's temporary linear
    // 20.5V (0%) to 29.2V (100%) model.
    private_nh_.param("battery_voltage", battery_voltage_, 28.765);
    private_nh_.param("motor_current", motor_current_, 0.0);
    private_nh_.param("motor_temperature", motor_temperature_, 25.0);

    joint_to_motor_id_["front_right_wheel"] =
        scout_msgs::ScoutStatus::MOTOR_ID_FRONT_RIGHT;
    joint_to_motor_id_["front_left_wheel"] =
        scout_msgs::ScoutStatus::MOTOR_ID_FRONT_LEFT;
    joint_to_motor_id_["rear_right_wheel"] =
        scout_msgs::ScoutStatus::MOTOR_ID_REAR_RIGHT;
    joint_to_motor_id_["rear_left_wheel"] =
        scout_msgs::ScoutStatus::MOTOR_ID_REAR_LEFT;

    for (int i = 0; i < 4; ++i) {
      motor_rpm_[i] = 0.0;
    }

    light_control_enabled_ = false;
    front_light_mode_ = scout_msgs::ScoutLightCmd::LIGHT_CONST_OFF;
    rear_light_mode_ = scout_msgs::ScoutLightCmd::LIGHT_CONST_OFF;
    front_light_custom_value_ = 0;
    rear_light_custom_value_ = 0;

    status_pub_ = nh_.advertise<scout_msgs::ScoutStatus>(status_topic_, 10);
    status_text_pub_ = nh_.advertise<std_msgs::String>(status_text_topic_, 10);
    odom_sub_ =
        nh_.subscribe(odom_topic_, 10, &SimScoutStatus::OdomCallback, this);
    joint_state_sub_ = nh_.subscribe(joint_states_topic_, 10,
                                     &SimScoutStatus::JointStateCallback, this);
    light_cmd_sub_ = nh_.subscribe(light_control_topic_, 5,
                                   &SimScoutStatus::LightCmdCallback, this);

    if (publish_rate_ <= 0.0) {
      publish_rate_ = 50.0;
    }
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                             &SimScoutStatus::PublishStatus, this);

    ROS_INFO("Sim Scout status: odom=%s joint_states=%s status=%s status_text=%s light_cmd=%s",
             odom_topic_.c_str(), joint_states_topic_.c_str(),
             status_topic_.c_str(), status_text_topic_.c_str(),
             light_control_topic_.c_str());
  }

 private:
  static std::string DeriveStatusTextTopic(const std::string& status_topic) {
    const std::string suffix = "scout_status";
    if (status_topic.size() >= suffix.size() &&
        status_topic.compare(status_topic.size() - suffix.size(), suffix.size(),
                             suffix) == 0) {
      return status_topic.substr(0, status_topic.size() - suffix.size()) +
             "scout/status_text";
    }
    if (!status_topic.empty() && status_topic.back() == '/') {
      return status_topic + "scout/status_text";
    }
    return status_topic + "/scout/status_text";
  }

  static bool EndsWith(const std::string& value, const std::string& suffix) {
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) ==
               0;
  }

  int MotorIdForJoint(const std::string& joint_name) const {
    std::map<std::string, int>::const_iterator it = joint_to_motor_id_.begin();
    for (; it != joint_to_motor_id_.end(); ++it) {
      if (joint_name == it->first || EndsWith(joint_name, "/" + it->first)) {
        return it->second;
      }
    }
    return -1;
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    latest_odom_ = *msg;
    have_odom_ = true;
  }

  void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    const double rad_per_sec_to_rpm = 60.0 / (2.0 * 3.14159265358979323846);
    for (std::size_t i = 0; i < msg->name.size() && i < msg->velocity.size();
         ++i) {
      int motor_id = MotorIdForJoint(msg->name[i]);
      if (motor_id >= 0 && motor_id < 4) {
        motor_rpm_[motor_id] = msg->velocity[i] * rad_per_sec_to_rpm;
      }
    }
  }

  void LightCmdCallback(const scout_msgs::ScoutLightCmd::ConstPtr& msg) {
    light_control_enabled_ = msg->enable_cmd_light_control;
    front_light_mode_ = msg->front_mode;
    front_light_custom_value_ = msg->front_custom_value;
    rear_light_mode_ = msg->rear_mode;
    rear_light_custom_value_ = msg->rear_custom_value;
  }

  void PublishStatus(const ros::TimerEvent&) {
    scout_msgs::ScoutStatus status_msg;
    status_msg.header.stamp = ros::Time::now();

    if (have_odom_) {
      status_msg.linear_velocity = latest_odom_.twist.twist.linear.x;
      status_msg.angular_velocity = latest_odom_.twist.twist.angular.z;
    } else {
      status_msg.linear_velocity = 0.0;
      status_msg.angular_velocity = 0.0;
    }

    status_msg.base_state = static_cast<uint8_t>(base_state_);
    status_msg.control_mode = static_cast<uint8_t>(control_mode_);
    status_msg.fault_code = static_cast<uint16_t>(fault_code_);
    status_msg.battery_voltage = battery_voltage_;

    for (int i = 0; i < 4; ++i) {
      status_msg.motor_states[i].current = motor_current_;
      status_msg.motor_states[i].rpm = motor_rpm_[i];
      status_msg.motor_states[i].temperature = motor_temperature_;
    }

    status_msg.light_control_enabled = light_control_enabled_;
    status_msg.front_light_state.mode = front_light_mode_;
    status_msg.front_light_state.custom_value = front_light_custom_value_;
    status_msg.rear_light_state.mode = rear_light_mode_;
    status_msg.rear_light_state.custom_value = rear_light_custom_value_;

    status_pub_.publish(status_msg);

    char buf[160];
    std::snprintf(
        buf, sizeof(buf),
        "mode=%u base=%u fault=%u batt=%.2f light_mode=%u light=%u",
        static_cast<unsigned>(status_msg.control_mode),
        static_cast<unsigned>(status_msg.base_state),
        static_cast<unsigned>(status_msg.fault_code),
        status_msg.battery_voltage,
        static_cast<unsigned>(status_msg.front_light_state.mode),
        static_cast<unsigned>(status_msg.front_light_state.custom_value));
    std_msgs::String text;
    text.data = buf;
    status_text_pub_.publish(text);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher status_pub_;
  ros::Publisher status_text_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber light_cmd_sub_;
  ros::Timer timer_;

  std::string odom_topic_;
  std::string joint_states_topic_;
  std::string status_topic_;
  std::string status_text_topic_;
  std::string light_control_topic_;
  double publish_rate_;
  int base_state_;
  int control_mode_;
  int fault_code_;
  double battery_voltage_;
  double motor_current_;
  double motor_temperature_;

  nav_msgs::Odometry latest_odom_;
  bool have_odom_;
  double motor_rpm_[4];
  std::map<std::string, int> joint_to_motor_id_;

  bool light_control_enabled_;
  uint8_t front_light_mode_;
  uint8_t rear_light_mode_;
  uint8_t front_light_custom_value_;
  uint8_t rear_light_custom_value_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_scout_status");
  SimScoutStatus node;
  ros::spin();
  return 0;
}
