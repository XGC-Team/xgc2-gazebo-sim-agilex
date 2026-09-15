#include <xgc2_math/control/wheel_drive.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/Float64.h>
#include <urdf/model.h>

namespace scout_gazebo {

// Wheel-drive identification candidate: I acts on target error; P acts on
// measured velocity. The controller manager supplies the physics-step period.
// No setpoint filter, command delay, or extra actuator pole is introduced.
class WheelVelocityController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root,
            ros::NodeHandle& node) override {
    std::string name;
    if (!node.getParam("joint", name) || !node.getParam("p", p_) ||
        !node.getParam("i", i_) || !std::isfinite(p_) || !std::isfinite(i_) ||
        p_ < 0 || i_ <= 0) return false;
    urdf::Model model;
    if (!model.initParamWithNodeHandle("robot_description", root)) return false;
    const auto joint = model.getJoint(name);
    if (!joint || !joint->limits) return false;
    effort_limit_ = joint->limits->effort;
    velocity_limit_ = joint->limits->velocity;
    if (!(effort_limit_ > 0 && velocity_limit_ > 0)) return false;
    joint_ = hw->getHandle(name);
    target_.store(0.0);
    subscriber_ = node.subscribe<std_msgs::Float64>("command", 1,
        &WheelVelocityController::command, this);
    return true;
  }

  void starting(const ros::Time&) override {
    state_ = {0.0, joint_.getVelocity()};
    joint_.setCommand(0);
  }

  void update(const ros::Time&, const ros::Duration& period) override {
    const double velocity = joint_.getVelocity();
    const double dt = period.toSec();
    if (dt <= 0) return;
    state_ = xgc2_math::wheelVelocityIPStep(
        state_, target_.load(), velocity, dt, {p_, i_, effort_limit_});
    joint_.setCommand(state_.effort_nm);
  }

  void stopping(const ros::Time&) override {
    state_ = {};
    target_.store(0);
    joint_.setCommand(0);
  }

 private:
  void command(const std_msgs::Float64::ConstPtr& message) {
    if (std::isfinite(message->data))
      target_.store(std::max(-velocity_limit_, std::min(velocity_limit_, message->data)));
  }
  hardware_interface::JointHandle joint_;
  ros::Subscriber subscriber_;
  std::atomic<double> target_{0};
  double p_=0, i_=0, effort_limit_=0, velocity_limit_=0;
  xgc2_math::WheelVelocityIPState state_;
};
}  // namespace scout_gazebo

PLUGINLIB_EXPORT_CLASS(scout_gazebo::WheelVelocityController, controller_interface::ControllerBase)
