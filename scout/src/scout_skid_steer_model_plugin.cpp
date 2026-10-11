#include <atomic>
#include <xgc2_gazebo_scene/ros_master_registration.hpp>
#include "scout_gazebo/scout_skid_steer.hpp"
#include <gazebo/common/Plugin.hh>
#include <stdexcept>
namespace scout_gazebo {
class SkidSteerModelPlugin final : public gazebo::ModelPlugin {
 public:
  ~SkidSteerModelPlugin() override { registration_.reset(); }
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr config) override {
    if (!ros::isInitialized()) throw std::runtime_error("native robot requires the Gazebo ROS data owner");
    registration_ = std::make_unique<xgc2_gazebo_scene::RosMasterRegistration>([this, model, config] {
      Initialize(model, config);
      initialized_.store(true, std::memory_order_release);
    });
  }
  void Initialize(gazebo::physics::ModelPtr model, sdf::ElementPtr config) {
    if (!ros::isInitialized()) throw std::runtime_error("Scout native skid requires Gazebo ROS data owner");
    // This model exclusively owns wheel efforts. A second ROS control plugin
    // would make native HOLD completion impossible to attest.
    const auto description = model->GetSDF();
    if (description->HasElement("plugin"))
      for (auto plugin=description->GetElement("plugin"); plugin; plugin=plugin->GetNextElement("plugin"))
        if (plugin->Get<std::string>("filename") == "libgazebo_ros_control.so")
          throw std::runtime_error("native Scout skid cannot share wheel effort ownership");
    node_.reset(new ros::NodeHandle(config->Get<std::string>("robotNamespace")));
    controller_.reset(new wescore::ScoutSkidSteer(node_.get(), model, config->Get<std::string>("chassisRobotId")));
    controller_->SetupSubscription();
  }
  void Reset() override { if (initialized_.load(std::memory_order_acquire) && controller_) controller_->Reset(); }
 private:
  std::atomic<bool> initialized_{false};
  std::unique_ptr<xgc2_gazebo_scene::RosMasterRegistration> registration_;
  std::unique_ptr<ros::NodeHandle> node_;
  std::unique_ptr<wescore::ScoutSkidSteer> controller_;
};
GZ_REGISTER_MODEL_PLUGIN(SkidSteerModelPlugin)
}
