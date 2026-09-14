// Native Gazebo 11.15.1/ODE diagnostic, not a replacement production controller.
// Replays a frozen SDF and timestamped commands using the production delay core.
// The PI recurrence matches the audited control_toolbox 1.19.0 reference path;
// equivalence to an installed ROS plugin requires its separate package receipt.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <yaml-cpp/yaml.h>
#include "scout_gazebo/command_delay.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
double required(const YAML::Node &node, const std::string &key) {
  const double value=node[key].as<double>();
  if (!std::isfinite(value)) throw std::runtime_error("Nonfinite YAML: "+key);
  return value;
}
double bounded(double value, double limit) {
  return std::clamp(value,-limit,limit);
}
struct Command { double time; double linear; double angular; };
std::vector<Command> readCommands(const std::string &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot read commands: "+path);
  std::string line;
  if (!std::getline(input,line) || line!="time_s,linear_m_s,angular_rad_s")
    throw std::runtime_error("Command CSV must be time_s,linear_m_s,angular_rad_s");
  std::vector<Command> commands;
  while (std::getline(input,line)) {
    if (line.empty()) continue;
    std::replace(line.begin(),line.end(),',',' ');
    std::istringstream stream(line);
    Command command{};std::string extra;
    if (!(stream>>command.time>>command.linear>>command.angular) || (stream>>extra) ||
        !std::isfinite(command.time) || !std::isfinite(command.linear) ||
        !std::isfinite(command.angular) || command.time<0 ||
        (!commands.empty() && command.time<commands.back().time))
      throw std::runtime_error("Invalid/nonmonotone command CSV row");
    commands.push_back(command);
  }
  if (commands.empty()) throw std::runtime_error("Empty command stream");
  return commands;
}
struct Pi {
  double kp,ki,integral_min,integral_max,effort_limit;
  double integral_error=0;
  double output(double error, double h) {
    // antiwindup=true in the frozen profiles. The integral-error state is
    // clamped before total torque saturation; no hidden back-calculation.
    integral_error+=error*h;
    if (ki>0) integral_error=std::clamp(integral_error,integral_min/ki,integral_max/ki);
    return bounded(kp*error+ki*integral_error,effort_limit);
  }
};
class Replay {
 public:
  explicit Replay(const YAML::Node &config):
      config_(config),commands_(readCommands(config["commands_csv"].as<std::string>())),
      output_(config["output_csv"].as<std::string>()) {
    h_=required(config,"physics_step_s");
    duration_=required(config,"duration_s");
    read_period_=required(config,"controller_read_period_s");
    radius_=required(config,"wheel_radius_m");
    separation_=required(config,"allocation_separation_m");
    gain_=required(config,"command_gain");
    angular_gain_=required(config,"angular_command_gain");
    vmax_=required(config,"linear_command_limit_m_s");
    wmax_=required(config,"angular_command_limit_rad_s");
    wheel_max_=required(config,"wheel_velocity_limit_rad_s");
    const double delay=required(config,"communication_delay_s");
    if (delay!=0.005 || h_<=0 || h_>.01 || duration_<=0 || duration_>600 ||
        read_period_<h_ || radius_<=0 || separation_<=0 || gain_<=0 ||
        angular_gain_<=0 || vmax_<=0 || wmax_<=0 || wheel_max_<=0 || !output_)
      throw std::runtime_error("Invalid replay contract (delay must remain 0.005 s)");
    delay_.Configure(delay);
    const auto pi=config["wheel_pi"];
    const Pi p{required(pi,"p"),required(pi,"i"),required(pi,"i_min_Nm"),
               required(pi,"i_max_Nm"),required(pi,"effort_limit_Nm"),0};
    if (p.kp<0 || p.ki<=0 || p.integral_min>0 || p.integral_max<0 ||
        p.integral_min>p.integral_max || p.effort_limit<=0)
      throw std::runtime_error("Invalid wheel PI parameters");
    pi_.fill(p);
    const auto names=config["wheel_joint_names_FR_FL_RL_RR"].as<std::vector<std::string>>();
    if (names.size()!=4) throw std::runtime_error("Exactly four wheel joints required");
    std::copy(names.begin(),names.end(),names_.begin());
  }
  void run() {
    world_=gazebo::loadWorld(config_["world_sdf"].as<std::string>());
    if (!world_) throw std::runtime_error("Cannot load frozen world");
    if (std::abs(world_->Physics()->GetMaxStepSize()-h_)>1e-12)
      throw std::runtime_error("YAML and SDF physics steps differ");
    model_=world_->ModelByName(config_["model_name"].as<std::string>());
    if (!model_) throw std::runtime_error("Frozen model missing");
    base_=model_->GetLink(config_["base_link_name"].as<std::string>());
    if (!base_) throw std::runtime_error("Frozen base link missing");
    for (unsigned i=0;i<4;++i) {
      wheels_[i]=model_->GetJoint(names_[i]);
      if (!wheels_[i]) throw std::runtime_error("Wheel joint missing: "+names_[i]);
    }
    output_<<std::setprecision(17)
      <<"sim_time,x,y,z,yaw,v_parallel,v_perp,wz,cog_vx,cog_vy";
    for (const auto &name:names_) output_<<","<<name<<"_target,"<<name<<"_velocity,"<<name<<"_effort,"<<name<<"_integral_Nm";
    output_<<"\n";
    auto begin=gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo &info){stepBegin(info.simTime.Double());});
    auto end=gazebo::event::Events::ConnectWorldUpdateEnd([this](){stepEnd();});
    gazebo::runWorld(world_,static_cast<unsigned>(std::ceil(duration_/h_)));
    begin.reset();end.reset();output_.flush();
    if (!output_ || !finite_ || samples_==0) throw std::runtime_error("Replay has invalid or missing output");
    std::cout<<"{\"execution\":\"completed\",\"acceptance\":\"not_evaluated\",\"samples\":"
      <<samples_<<",\"physics_step_s\":"<<h_<<",\"communication_delay_s\":0.005,"
      <<"\"input_rows_consumed\":"<<next_<<"}\n";
  }
 private:
  void stepBegin(double time) {
    // The native receiver observes inputs on the simulation grid. Their
    // recorded arrival time is retained; quantization is not fitted away.
    while (next_<commands_.size() && commands_[next_].time<=time+1e-12) {
      const auto &command=commands_[next_++];
      delay_.Push(time,bounded(command.linear,vmax_),bounded(command.angular,wmax_));
    }
    const auto command=delay_.Advance(time);
    const double half=.5*separation_*angular_gain_*command.angular;
    const double left=bounded(gain_*(command.linear-half)/radius_,wheel_max_);
    const double right=bounded(gain_*(command.linear+half)/radius_,wheel_max_);
    pending_={right,left,left,right};
    // gazebo_ros_control reads/controller-updates on its control period, but
    // DefaultRobotHWSim writes PI effort every physics update using the last
    // cached joint velocity and desired velocity. Make the two clocks explicit.
    if (time-last_read_+1e-12>=read_period_) {
      for (unsigned i=0;i<4;++i) measured_[i]=wheels_[i]->GetVelocity(0);
      target_=pending_;last_read_=time;
    }
    for (unsigned i=0;i<4;++i) {
      effort_[i]=pi_[i].output(target_[i]-measured_[i],h_);
      wheels_[i]->SetForce(0,effort_[i]);
    }
  }
  void stepEnd() {
    const auto pose=base_->WorldPose();
    const auto velocity=base_->WorldLinearVel();
    const auto cog_velocity=base_->WorldCoGLinearVel();
    const double yaw=pose.Rot().Yaw();
    const double longitudinal=std::cos(yaw)*velocity.X()+std::sin(yaw)*velocity.Y();
    const double lateral=-std::sin(yaw)*velocity.X()+std::cos(yaw)*velocity.Y();
    const double omega=base_->WorldAngularVel().Z();
    finite_=finite_ && pose.IsFinite() && velocity.IsFinite() && std::isfinite(omega);
    output_<<world_->SimTime().Double()<<","<<pose.Pos().X()<<","<<pose.Pos().Y()<<","<<pose.Pos().Z()
      <<","<<yaw<<","<<longitudinal<<","<<lateral<<","<<omega
      <<","<<cog_velocity.X()<<","<<cog_velocity.Y();
    for (unsigned i=0;i<4;++i) output_<<","<<target_[i]<<","<<wheels_[i]->GetVelocity(0)
      <<","<<effort_[i]<<","<<pi_[i].ki*pi_[i].integral_error;
    output_<<"\n";++samples_;
  }
  YAML::Node config_;
  std::vector<Command> commands_;
  std::ofstream output_;
  std::array<std::string,4> names_{};
  std::array<Pi,4> pi_{};
  std::array<double,4> pending_{},target_{},measured_{},effort_{};
  std::array<gazebo::physics::JointPtr,4> wheels_{};
  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_;
  wescore::CommandDelay delay_;
  double h_=0,duration_=0,read_period_=0,radius_=0,separation_=0,gain_=0,angular_gain_=0;
  double vmax_=0,wmax_=0,wheel_max_=0,last_read_=0;
  std::size_t next_=0,samples_=0;
  bool finite_=true;
};
}
int main(int argc,char **argv) {
  if (argc!=2) {std::cerr<<"Usage: scout_native_replay frozen.yaml\n";return 2;}
  bool started=false;
  try {
    const auto config=YAML::LoadFile(argv[1]);
    started=gazebo::setupServer(std::vector<std::string>{"scout_native_replay"});
    if (!started) throw std::runtime_error("Cannot start isolated Gazebo server");
    {Replay replay(config);replay.run();}
    gazebo::shutdown();return 0;
  } catch (const std::exception &error) {
    std::cerr<<error.what()<<"\n";
    if (started) gazebo::shutdown();
    return 1;
  }
}
