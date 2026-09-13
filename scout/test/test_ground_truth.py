#!/usr/bin/env python3
"""Replay the production ground-truth callback with small ROS API doubles.

No ROS master or Gazebo is required. This covers callback/data contracts, not
ROS transport scheduling, source-time synchronization, or physical acceptance.
CXX and CXXFLAGS can select the compiler and enable sanitizers.
"""
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]

ROS_DOUBLES = r'''
#pragma once
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace ros {
static double now_seconds = 0.0;
class Time {
 public:
  explicit Time(double seconds = 0.0) : seconds_(seconds) {}
  static Time now() { return Time(now_seconds); }
  bool isZero() const { return seconds_ == 0.0; }
  bool operator==(const Time &other) const { return seconds_ == other.seconds_; }
  bool operator<(const Time &other) const { return seconds_ < other.seconds_; }
  bool operator<=(const Time &other) const { return seconds_ <= other.seconds_; }
 private:
  double seconds_;
};
}
namespace std_msgs {
struct Header { ros::Time stamp; std::string frame_id; };
}
namespace geometry_msgs {
struct Vector3 { double x = 0.0, y = 0.0, z = 0.0; };
struct Quaternion { double x = 0.0, y = 0.0, z = 0.0, w = 0.0; };
struct Pose { Vector3 position; Quaternion orientation; };
struct Twist { Vector3 linear, angular; };
struct PoseStamped { std_msgs::Header header; Pose pose; };
struct TwistStamped { std_msgs::Header header; Twist twist; };
}
namespace gazebo_msgs {
struct ModelStates {
  using ConstPtr = std::shared_ptr<const ModelStates>;
  std::vector<std::string> name;
  std::vector<geometry_msgs::Pose> pose;
  std::vector<geometry_msgs::Twist> twist;
};
}
namespace ros {
static std::vector<geometry_msgs::PoseStamped> poses;
static std::vector<geometry_msgs::TwistStamped> twists;
static std::function<void(const gazebo_msgs::ModelStates::ConstPtr &)> callback;
struct Subscriber {};
struct Publisher {
  void publish(const geometry_msgs::PoseStamped &message) { poses.push_back(message); }
  void publish(const geometry_msgs::TwistStamped &message) { twists.push_back(message); }
};
class NodeHandle {
 public:
  explicit NodeHandle(const std::string & = "") {}
  template <typename T>
  void param(const std::string &, T &value, const T &fallback) { value = fallback; }
  template <typename T>
  Publisher advertise(const std::string &, unsigned int) { return Publisher(); }
  template <typename T>
  Subscriber subscribe(const std::string &, unsigned int,
      void (T::*method)(const gazebo_msgs::ModelStates::ConstPtr &), T *owner) {
    callback = [owner, method](const gazebo_msgs::ModelStates::ConstPtr &message) {
      (owner->*method)(message);
    };
    return Subscriber();
  }
};
inline void init(int, char **, const char *) {}
inline void spin() {}
}
#define ROS_WARN_THROTTLE(...) do {} while (0)
'''

REPLAY = r'''
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

gazebo_msgs::ModelStates sample() {
  gazebo_msgs::ModelStates message;
  message.name = {"obstacle", "scout"};
  message.pose.resize(2);
  message.twist.resize(2);
  auto &pose = message.pose[1];
  pose.position.x = 1.0; pose.position.y = 2.0; pose.position.z = 3.0;
  pose.orientation.z = 0.6; pose.orientation.w = 0.8;
  auto &twist = message.twist[1];
  twist.linear.x = 0.4; twist.linear.y = -0.2; twist.linear.z = 0.03;
  twist.angular.x = 0.01; twist.angular.y = 0.02; twist.angular.z = -0.3;
  return message;
}

std::vector<double *> fields(geometry_msgs::Pose &p, geometry_msgs::Twist &t) {
  return {&p.position.x, &p.position.y, &p.position.z,
          &p.orientation.x, &p.orientation.y, &p.orientation.z, &p.orientation.w,
          &t.linear.x, &t.linear.y, &t.linear.z,
          &t.angular.x, &t.angular.y, &t.angular.z};
}

void deliver(double stamp, const gazebo_msgs::ModelStates &message) {
  ros::now_seconds = stamp;
  ros::callback(std::make_shared<const gazebo_msgs::ModelStates>(message));
}

void count(std::size_t expected) {
  require(ros::poses.size() == expected, "unexpected pose publication count");
  require(ros::twists.size() == expected, "unexpected twist publication count");
}

int main(int argc, char **argv) {
  try {
    require(argc == 2, "one replay scenario is required");
    const std::string scenario = argv[1];
    ros::NodeHandle private_nh("~");
    GazeboModelGroundTruth node(private_nh);
    auto good = sample();
    if (scenario == "normal") {
      deliver(10.0, good);
      count(1);
      require(ros::poses[0].header.stamp == ros::Time(10.0), "pose stamp changed");
      require(ros::twists[0].header.stamp == ros::poses[0].header.stamp,
              "pose and twist must share the same stamp");
      require(ros::poses[0].header.frame_id == "world" &&
              ros::twists[0].header.frame_id == "world", "world frame changed");
      auto expected = fields(good.pose[1], good.twist[1]);
      auto actual = fields(ros::poses[0].pose, ros::twists[0].twist);
      for (std::size_t i = 0; i < expected.size(); ++i)
        require(*expected[i] == *actual[i], "selected model/world-frame data changed");
    } else if (scenario == "duplicate") {
      deliver(10.0, good); deliver(10.0, good); count(1);
      deliver(10.5, good); count(2);
    } else if (scenario == "zero_start") {
      deliver(0.0, good); deliver(0.0, good); count(0);
      deliver(0.25, good); count(1);
    } else if (scenario == "positive_rewind") {
      deliver(100.0, good); deliver(0.25, good); count(2);
      deliver(0.25, good); count(2);
      deliver(0.5, good); count(3);
      require(ros::poses[1].header.stamp == ros::Time(0.25),
              "rewind must not relabel samples using the previous epoch");
    } else if (scenario == "zero_reset") {
      deliver(10.0, good); deliver(0.0, good); count(1);
      deliver(0.25, good); count(2);
    } else if (scenario == "zero_then_old_stamp") {
      deliver(10.0, good); deliver(0.0, good); deliver(10.0, good); count(2);
    } else if (scenario == "missing_model") {
      auto missing = good; missing.name[1] = "another_robot";
      deliver(1.0, missing); count(0);
      deliver(1.0, good); count(1);
    } else if (scenario == "rewind_while_missing") {
      auto missing = good; missing.name[1] = "another_robot";
      deliver(10.0, good); deliver(5.0, missing); count(1);
      deliver(10.0, good); count(2);
    } else if (scenario == "rewind_after_rejected_sample") {
      auto missing = good; missing.name[1] = "another_robot";
      deliver(10.0, good); deliver(20.0, missing); count(1);
      deliver(10.0, good); count(2);
      auto malformed = good; malformed.pose.clear();
      deliver(30.0, malformed); count(2);
      deliver(10.0, good); count(3);
    } else if (scenario == "malformed_arrays") {
      for (int variant = 0; variant < 4; ++variant) {
        auto bad = good;
        if (variant == 0) { bad.pose.resize(1); bad.pose.shrink_to_fit(); }
        if (variant == 1) { bad.twist.resize(1); bad.twist.shrink_to_fit(); }
        if (variant == 2) bad.pose.resize(3);
        if (variant == 3) bad.twist.resize(3);
        deliver(variant + 1.0, bad); count(variant);
        deliver(variant + 1.0, good); count(variant + 1);
      }
    } else if (scenario == "nonfinite") {
      const double invalid[] = {std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::infinity(),
                                -std::numeric_limits<double>::infinity()};
      std::size_t accepted = 0;
      for (double value : invalid) {
        for (std::size_t index = 0; index < 13; ++index) {
          auto bad = good;
          *fields(bad.pose[1], bad.twist[1])[index] = value;
          deliver(accepted + 1.0, bad); count(accepted);
          deliver(accepted + 1.0, good); count(++accepted);
        }
      }
    } else if (scenario == "invalid_quaternion") {
      for (int variant = 0; variant < 2; ++variant) {
        auto bad = good;
        bad.pose[1].orientation = geometry_msgs::Quaternion();
        if (variant == 1) bad.pose[1].orientation.w = 1e300;
        deliver(variant + 1.0, bad); count(variant);
        deliver(variant + 1.0, good); count(variant + 1);
      }
    } else if (scenario == "unselected_invalid_model") {
      good.pose[0].position.x = std::numeric_limits<double>::quiet_NaN();
      deliver(1.0, good); count(1);
    } else {
      throw std::runtime_error("unknown replay scenario");
    }
    std::cout << scenario << " passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
'''


class GroundTruthTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        temporary = tempfile.TemporaryDirectory(prefix="scout-ground-truth-")
        cls.addClassCleanup(temporary.cleanup)
        root = Path(temporary.name)
        (root / "test_ros.hpp").write_text(ROS_DOUBLES)
        for header in ("ros/ros.h", "gazebo_msgs/ModelStates.h",
                       "geometry_msgs/PoseStamped.h", "geometry_msgs/TwistStamped.h"):
            path = root / header
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('#include "test_ros.hpp"\n')
        source = root / "replay.cpp"
        source.write_text(
            '#define main ground_truth_node_main\n#include '
            + json.dumps(str(PACKAGE / "src/gazebo_model_ground_truth.cpp"))
            + '\n#undef main\n' + REPLAY
        )
        cls.executable = root / "replay"
        subprocess.run(
            [os.environ.get("CXX", "g++"), "-std=c++11", "-Wall", "-Wextra",
             "-Werror", "-pedantic"] + shlex.split(os.environ.get("CXXFLAGS", ""))
            + ["-I" + str(root), str(source), "-o", str(cls.executable)],
            check=True, timeout=60,
        )

    def replay(self, scenario):
        result = subprocess.run([str(self.executable), scenario], text=True,
                                capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0,
                         "{}: {}{}".format(scenario, result.stdout, result.stderr))

    def test_selected_model_pair_preserves_world_frame_and_stamp(self):
        self.replay("normal")

    def test_paused_positive_clock_deduplicates(self):
        self.replay("duplicate")

    def test_zero_clock_is_not_a_measurement_stamp(self):
        self.replay("zero_start")

    def test_positive_clock_rewind_recovers_immediately(self):
        self.replay("positive_rewind")

    def test_reset_via_zero_recovers_immediately(self):
        self.replay("zero_reset")

    def test_zero_observation_clears_previous_epoch(self):
        self.replay("zero_then_old_stamp")

    def test_missing_model_does_not_consume_stamp(self):
        self.replay("missing_model")

    def test_rewind_is_observed_while_model_is_missing(self):
        self.replay("rewind_while_missing")

    def test_clock_observation_is_independent_of_sample_acceptance(self):
        self.replay("rewind_after_rejected_sample")

    def test_malformed_arrays_do_not_publish_or_consume_stamp(self):
        self.replay("malformed_arrays")

    def test_all_pose_and_twist_fields_reject_nonfinite_values(self):
        self.replay("nonfinite")

    def test_invalid_quaternion_does_not_publish_or_consume_stamp(self):
        self.replay("invalid_quaternion")

    def test_other_models_do_not_invalidate_selected_model(self):
        self.replay("unselected_invalid_model")


if __name__ == "__main__":
    unittest.main()
