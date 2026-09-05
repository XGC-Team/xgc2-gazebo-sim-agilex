from pathlib import Path
import subprocess


def replace(path, old, new, count=1):
    file = Path(path)
    text = file.read_text()
    if text.count(old) != count:
        raise RuntimeError(f'{path}: expected {count} occurrences, found {text.count(old)}')
    file.write_text(text.replace(old, new))


def replace_region(path, start, end, new):
    file = Path(path)
    text = file.read_text()
    if text.count(start) != 1 or text.count(end) != 1:
        raise RuntimeError(f'{path}: ambiguous region')
    a, b = text.index(start), text.index(end)
    if b <= a:
        raise RuntimeError(f'{path}: reversed region')
    file.write_text(text[:a] + new + text[b:])


udp = 'scout/include/xgc_chassis_hold/udp.hpp'
replace(udp, '#include <thread>\n', '#include <thread>\n\n#include "xgc_chassis_hold/gate.hpp"\n')
replace_region(udp, 'class Gate {', 'inline void writeU32LE', '')
replace_region(udp, '  void add(Gate *gate) {', '\n private:\n  Hub()', '''  void add(Gate *gate) {
    if (gate == nullptr) return;
    start();
    registry_.add(gate);
  }

  void remove(Gate *gate) { registry_.remove(gate); }
''')
replace(udp, '''      close(fd_);
      fd_ = -1;
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }''', '''    }
    if (thread_.joinable()) {
      thread_.join();
    }
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }''')
replace(udp, '''      Gate *gate = match(robot);
      if (gate != nullptr) {
        gate->setHeld(held);
      }''', '''      const bool matched = registry_.apply(robot, held);''')
replace(udp, 'ack[6] = gate != nullptr ? 0 : 1;', 'ack[6] = matched ? 0 : 1;')
replace_region(udp, '  Gate *match(const char *robot_id) {', '  std::atomic<bool> started_', '  GateRegistry registry_;\n')

header = 'scout/include/scout_gazebo/scout_skid_steer.hpp'
replace(header, '#include <deque>\n', '')
replace(header, '#include "xgc_chassis_hold/udp.hpp"', '#include "xgc_chassis_hold/udp.hpp"\n#include "scout_gazebo/command_dynamics.hpp"')
replace_region(header, '  struct CommandSample {', '  ros::NodeHandle *nh_;', '  CommandDynamics command_dynamics_;\n\n')
replace(header, '  ros::Subscriber cmd_sub_;', '  ros::Subscriber cmd_sub_;\n  ros::WallTimer control_timer_;')
replace_region(header, '  void UpdateCommandDynamics(', '  double Clamp(', '  void ControlTick(const ros::WallTimerEvent &event);\n')

source = 'scout/src/scout_skid_steer.cpp'
replace(source, '''      command_time_constant_s_(0.15), delayed_linear_velocity_(0.0),
      delayed_angular_velocity_(0.0), filtered_linear_velocity_(0.0),
      filtered_angular_velocity_(0.0), nh_(nh),''', '''      command_time_constant_s_(0.15), nh_(nh),''')
replace(source, '  motor_fr_topic_ = JoinTopic(', '''  if (!std::isfinite(wheel_radius_) || wheel_radius_ <= 0.0 ||
      !std::isfinite(wheel_separation_) || wheel_separation_ <= 0.0 ||
      !std::isfinite(command_gain_) || !std::isfinite(angular_command_gain_)) {
    throw std::invalid_argument("Scout wheel geometry and command gains must be finite; geometry must be positive");
  }
  command_dynamics_.Configure(command_delay_s_, command_time_constant_s_);

  motor_fr_topic_ = JoinTopic(''')
replace(source, '''ScoutSkidSteer::~ScoutSkidSteer() {
  xgc_chassis_hold::Hub::instance().remove(&hold_gate_);
}''', '''ScoutSkidSteer::~ScoutSkidSteer() {
  control_timer_.stop();
  cmd_sub_.shutdown();
  // remove() drains any UDP callback before the Gate and publishers die.
  xgc_chassis_hold::Hub::instance().remove(&hold_gate_);
}''')
replace_region(source, 'void ScoutSkidSteer::SetupSubscription() {', 'void ScoutSkidSteer::HoldZeroThunk(', '''void ScoutSkidSteer::SetupSubscription() {
  motor_fr_pub_ = nh_->advertise<std_msgs::Float64>(motor_fr_topic_, 50);
  motor_fl_pub_ = nh_->advertise<std_msgs::Float64>(motor_fl_topic_, 50);
  motor_rl_pub_ = nh_->advertise<std_msgs::Float64>(motor_rl_topic_, 50);
  motor_rr_pub_ = nh_->advertise<std_msgs::Float64>(motor_rr_topic_, 50);
  hold_gate_.setZeroThunk(&ScoutSkidSteer::HoldZeroThunk, this);
  xgc_chassis_hold::Hub::instance().add(&hold_gate_);
  cmd_sub_ = nh_->subscribe<geometry_msgs::Twist>(
      cmd_topic_, 5, &ScoutSkidSteer::TwistCmdCallback, this);
  // Wall scheduling survives a paused/rewound clock. The plant itself uses
  // ROS simulation time, so wall ticks never advance paused dynamics.
  control_timer_ = nh_->createWallTimer(
      ros::WallDuration(0.01), &ScoutSkidSteer::ControlTick, this);
}

''')
replace(source, 'void ScoutSkidSteer::PublishZeroMotors() {\n', 'void ScoutSkidSteer::PublishZeroMotors() {\n  command_dynamics_.Reset();\n')
replace(source, '''  if (hold_gate_.held()) {
    PublishZeroMotors();
    return;
  }
''', '')
replace_region(source, '  UpdateCommandDynamics(driving_vel, steering_vel,', 'double ScoutSkidSteer::Clamp(', '''  hold_gate_.withCommand([&](bool held) {
    if (held) {
      PublishZeroMotors();
      return;
    }
    command_dynamics_.Push(ros::Time::now().toSec(), driving_vel, steering_vel);
  });
}

void ScoutSkidSteer::ControlTick(const ros::WallTimerEvent &) {
  hold_gate_.withCommand([this](bool held) {
    if (held) {
      PublishZeroMotors();
      return;
    }
    const CommandVelocity command = command_dynamics_.Advance(ros::Time::now().toSec());
    const double steering = command.angular * angular_command_gain_;
    const double half_track = wheel_separation_ * 0.5;
    const double left = (command.linear - steering * half_track) / wheel_radius_;
    const double right = (command.linear + steering * half_track) / wheel_radius_;
    std_msgs::Float64 motor_cmd[4];
    motor_cmd[0].data = right * command_gain_;
    motor_cmd[1].data = left * command_gain_;
    motor_cmd[2].data = left * command_gain_;
    motor_cmd[3].data = right * command_gain_;
    motor_fr_pub_.publish(motor_cmd[0]);
    motor_fl_pub_.publish(motor_cmd[1]);
    motor_rl_pub_.publish(motor_cmd[2]);
    motor_rr_pub_.publish(motor_cmd[3]);
  });
}

''')
replace('scout/launch/spawn_accurate.launch', '              -unpause\n', '')

subprocess.run(['python3', '-m', 'unittest', 'discover', '-s', 'scout/test', '-p', 'test_*.py', '-v'], check=True)
for name in ('hold_gate', 'command_dynamics'):
    for suffix, flags in (('plain', ['-Wall', '-Wextra', '-Werror']),
                          ('san', ['-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer'])):
        binary = '/tmp/' + name + '_' + suffix
        subprocess.run(['g++', '-std=c++11', '-pthread', '-Iscout/include', *flags,
                        'scout/test/' + name + '_test.cpp', '-o', binary], check=True)
        subprocess.run([binary], check=True, timeout=30)
