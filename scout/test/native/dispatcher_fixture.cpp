#include <xgc2_gazebo_scene/chassis_ros_callbacks.hpp>
#include <xgc2_gazebo_scene/native_component_binding.hpp>
#include <xgc2_gazebo_scene/native_world_extension.hpp>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <condition_variable>
#include <mutex>
#include <atomic>

namespace {
using namespace xgc2_gazebo_scene;
class Barrier final : public NativeWorldExtension {
  struct Command final : PreparedWorldExtension { explicit Command(bool release_) : release(release_) {} bool release; };
 public:
  Json::Value Describe() const override { return Json::Value(Json::objectValue); }
  std::shared_ptr<const PreparedWorldExtension> Prepare(const std::string& method, const std::string& route,
      const Json::Value& body) override {
    const bool release = method == "POST" && route == "/v1/extensions/dispatcher-test/release";
    const bool status = method == "GET" && route == "/v1/extensions/dispatcher-test/state";
    const bool valid_body = body.isObject() && (status ? body.empty() :
        body.size() == 1 && body["operation_timeout_ms"].isUInt() &&
        body["operation_timeout_ms"].asUInt() > 0 && body["operation_timeout_ms"].asUInt() <= 5000);
    if ((!release && !status) || !valid_body)
      throw NativeExtensionError(400, "invalid_argument", "invalid dispatcher barrier command");
    return std::make_shared<Command>(release);
  }
  Json::Value Execute(const std::shared_ptr<const PreparedWorldExtension>& value, bool* started) override {
    if (std::static_pointer_cast<const Command>(value)->release) {
      *started = true; Stop();
    }
    Json::Value result; result["blocked"] = blocked.load(); return result;
  }
  void Block() {
    std::unique_lock<std::mutex> lock(mutex);
    blocked.store(true);
    condition.wait(lock, [this] { return released; });
    blocked.store(false);
  }
  void Stop() noexcept override {
    { std::lock_guard<std::mutex> lock(mutex); released = true; }
    condition.notify_all();
  }
  std::atomic<bool> blocked{false};
 private:
  std::mutex mutex; std::condition_variable condition; bool released = false;
};
class DispatcherFixture final : public gazebo::WorldPlugin {
 public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr) override {
    barrier_ = std::make_shared<Barrier>();
    component_ = std::make_unique<NativeComponentBinding>(world, "dispatcher-test");
    extension_ = std::make_unique<WorldExtensionBinding>(world, "dispatcher-test", barrier_);
    callbacks_ = std::make_unique<ChassisRosCallbacks>(world);
    node_ = std::make_unique<ros::NodeHandle>(); node_->setCallbackQueue(callbacks_->queue());
    command_ = node_->subscribe<std_msgs::Bool>("/fixture/block_dispatcher", 1,
        callbacks_->wrap([barrier=barrier_](const std_msgs::Bool::ConstPtr& value) { if (value->data) barrier->Block(); }));
    component_->Ready();
  }
  ~DispatcherFixture() override {
    if (barrier_) barrier_->Stop();
    command_.shutdown(); if (callbacks_) callbacks_->drain(); extension_.reset();
  }
 private:
  std::shared_ptr<Barrier> barrier_;
  std::unique_ptr<NativeComponentBinding> component_;
  std::unique_ptr<WorldExtensionBinding> extension_;
  std::unique_ptr<ChassisRosCallbacks> callbacks_;
  std::unique_ptr<ros::NodeHandle> node_;
  ros::Subscriber command_;
};
GZ_REGISTER_WORLD_PLUGIN(DispatcherFixture)
}
