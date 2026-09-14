// ROS-free open-loop reference using the ACTUAL pinned Gazebo vendored ODE.
// This is a mechanical/control reconstruction, not a Gazebo binary-equivalence
// claim. Its assumptions and conformance/physical errors must be reported.
#include <gazebo/ode/ode.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using boost::property_tree::ptree;
namespace {
std::vector<double> vec(const ptree& p, const std::string& key, std::size_t size) {
  std::vector<double> out;
  for (const auto& value : p.get_child(key)) out.push_back(value.second.get_value<double>());
  if (out.size() != size) throw std::runtime_error("wrong vector dimension: " + key);
  for (double value : out) if (!std::isfinite(value)) throw std::runtime_error("nonfinite config");
  return out;
}
double clip(double value, double limit) { return std::max(-limit, std::min(limit, value)); }
void rotation(dMatrix3 r, const std::vector<double>& pose) {
  // Rz(yaw) Ry(pitch) Rx(roll), body to world.
  const double c=std::cos(pose[3]),s=std::sin(pose[3]);
  const double cp=std::cos(pose[4]),sp=std::sin(pose[4]);
  const double cy=std::cos(pose[5]),sy=std::sin(pose[5]);
  r[0]=cy*cp;r[1]=cy*sp*s-sy*c;r[2]=cy*sp*c+sy*s;r[3]=0;
  r[4]=sy*cp;r[5]=sy*sp*s+cy*c;r[6]=sy*sp*c-cy*s;r[7]=0;
  r[8]=-sp;r[9]=cp*s;r[10]=cp*c;r[11]=0;
}
struct Sample { double t; std::array<double,4> value{}; };
std::vector<Sample> inputs(const std::string& path, bool wheel_mode) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open input CSV");
  std::string line; std::getline(stream,line);
  const std::string expected=wheel_mode ? "time_s,fr_radps,fl_radps,rl_radps,rr_radps" : "time_s,v_mps,w_radps";
  if (line != expected) throw std::runtime_error("unexpected input schema: " + line);
  std::vector<Sample> result;
  while(std::getline(stream,line)) {
    std::replace(line.begin(),line.end(),',',' '); std::istringstream row(line);
    Sample sample;
    if (!(row>>sample.t)) throw std::runtime_error("invalid input time");
    for(int k=0;k<(wheel_mode?4:2);++k)
      if (!(row>>sample.value[k]) || !std::isfinite(sample.value[k])) throw std::runtime_error("invalid input value");
    if (!std::isfinite(sample.t) || sample.t<0 || (!result.empty() && sample.t<result.back().t))
      throw std::runtime_error("input clock regression/nonfinite/negative; epochs must be independent");
    result.push_back(sample);
  }
  if(result.empty()) throw std::runtime_error("empty input");
  return result;
}
struct Surface {
  double mu,mu2,slip1,slip2,kp,kd,min_depth,max_vel;
  int max_contacts;
  std::array<double,3> fdir{};
  Surface(const ptree& p):mu(p.get<double>("mu")),mu2(p.get<double>("mu2")),
    slip1(p.get<double>("slip1")),slip2(p.get<double>("slip2")),kp(p.get<double>("kp")),
    kd(p.get<double>("kd")),min_depth(p.get<double>("min_depth")),max_vel(p.get<double>("max_vel")),
    max_contacts(p.get<int>("max_contacts")) {
    const auto v=vec(p,"fdir1",3);std::copy(v.begin(),v.end(),fdir.begin());
    if(kp<=0 || kd<0 || mu<0 || mu2<0 || slip1<0 || slip2<0 || max_contacts<1)
      throw std::runtime_error("invalid contact parameters");
  }
};
struct Collision {
  dGeomID geom=nullptr;
  int body_index=-1;
  Surface surface;
  Collision(int body,const ptree& p):body_index(body),surface(p.get_child("surface")){}
};
struct Motor {
  dJointID joint=nullptr;
  double p=0,i=0,integral=0,imax=0,effort=0,velocity=0,damping=0;
  bool antiwindup=true;
  double target=0,torque=0;
};
struct ContactReceipt { dJointFeedback feedback{}; int body=-1; std::array<double,3> point{}; };

class Plant {
 public:
  dWorldID world=dWorldCreate();
  dSpaceID space=dSimpleSpaceCreate(nullptr);
  dJointGroupID contacts=dJointGroupCreate(0);
  std::vector<dBodyID> bodies;
  std::deque<Collision> collisions;
  std::deque<ContactReceipt> receipts;
  std::array<Motor,4> motors;
  std::vector<double> base_com;
  double h, delay, gain, angular_gain, span, radius, vmax, wmax;
  bool wheel_mode;
  int global_max_contacts;
  double origin_z;
  explicit Plant(const ptree& cfg):
    h(cfg.get<double>("world.step_s")),delay(cfg.get<double>("command.delay_s")),
    gain(cfg.get<double>("command.gain")),angular_gain(cfg.get<double>("command.angular_gain")),
    span(cfg.get<double>("command.allocation_span_m")),radius(cfg.get<double>("command.radius_m")),
    vmax(cfg.get<double>("command.max_v_mps")),wmax(cfg.get<double>("command.max_w_radps")),
    wheel_mode(cfg.get<std::string>("input_mode")=="wheel_targets"),
    global_max_contacts(cfg.get<int>("world.max_contacts")),origin_z(cfg.get<double>("initial_base_height_m")) {
    if(h<=0 || h>.01 || delay!=.005 || radius<=0 || span<=0 || vmax<=0 || wmax<=0)
      throw std::runtime_error("invalid physical step/geometry/limits or non-frozen delay");
    dWorldSetGravity(world,0,0,cfg.get<double>("world.gravity_z_mps2"));
    dWorldSetCFM(world,cfg.get<double>("world.cfm"));
    dWorldSetERP(world,cfg.get<double>("world.erp"));
    dWorldSetQuickStepNumIterations(world,cfg.get<int>("world.iterations"));
    dWorldSetQuickStepW(world,cfg.get<double>("world.sor"));
    dWorldSetAutoDisableFlag(world,0);
    dWorldSetContactSurfaceLayer(world,cfg.get<double>("world.surface_layer_m"));
    dWorldSetContactMaxCorrectingVel(world,cfg.get<double>("world.max_correcting_mps"));
    const auto friction=cfg.get<std::string>("world.friction_model");
    if(friction!="cone_model" && friction!="pyramid_model") throw std::runtime_error("unsupported friction model");
    dWorldSetQuickStepFrictionModel(world,friction=="cone_model"?cone_friction:pyramid_friction);
    for(const auto& item:cfg.get_child("bodies")) {
      const ptree& p=item.second;
      const int index=static_cast<int>(bodies.size());
      dBodyID body=dBodyCreate(world); bodies.push_back(body);
      const auto pose=vec(p,"pose",6),com=vec(p,"com",3),inertia=vec(p,"inertia",6);
      if(index==0) base_com=com;
      dMatrix3 r;rotation(r,pose);dBodySetRotation(body,r);
      dBodySetPosition(body,pose[0]+r[0]*com[0]+r[1]*com[1]+r[2]*com[2],
        pose[1]+r[4]*com[0]+r[5]*com[1]+r[6]*com[2],
        origin_z+pose[2]+r[8]*com[0]+r[9]*com[1]+r[10]*com[2]);
      dMass mass;dMassSetParameters(&mass,p.get<double>("mass_kg"),0,0,0,
        inertia[0],inertia[1],inertia[2],inertia[3],inertia[4],inertia[5]);
      if(!dMassCheck(&mass)) throw std::runtime_error("invalid resolved inertia");
      dBodySetMass(body,&mass);dBodySetAutoDisableFlag(body,0);
      for(const auto& geometry:p.get_child("collisions")) {
        const ptree& c=geometry.second;
        collisions.emplace_back(index,c);auto& obj=collisions.back();
        const auto dims=vec(c,"dimensions",3),offset=vec(c,"pose",6);
        const auto shape=c.get<std::string>("shape");
        if(shape=="box") obj.geom=dCreateBox(space,dims[0],dims[1],dims[2]);
        else if(shape=="cylinder") obj.geom=dCreateCylinder(space,dims[0],dims[1]);
        else throw std::runtime_error("unsupported resolved collision primitive");
        dGeomSetBody(obj.geom,body);
        dGeomSetOffsetPosition(obj.geom,offset[0]-com[0],offset[1]-com[1],offset[2]-com[2]);
        dMatrix3 cr;rotation(cr,offset);dGeomSetOffsetRotation(obj.geom,cr);
        dGeomSetData(obj.geom,&obj);
        // Match ODELink::UpdateSurface: each collision updates body properties.
        dBodySetMaxVel(body,obj.surface.max_vel<0?cfg.get<double>("world.max_correcting_mps"):obj.surface.max_vel);
        dBodySetMinDepth(body,obj.surface.min_depth);
      }
    }
    if(bodies.size()!=5) throw std::runtime_error("expected resolved base plus four wheels");
    int mi=0;
    for(const auto& joint:cfg.get_child("motors")) {
      if(mi>=4) throw std::runtime_error("too many motors");
      const auto& p=joint.second;auto& motor=motors[mi];
      const auto anchor=vec(p,"anchor",3),axis=vec(p,"axis_world",3);
      motor.joint=dJointCreateHinge(world,nullptr);
      dJointAttach(motor.joint,bodies[mi+1],bodies[0]);
      dJointSetHingeAnchor(motor.joint,anchor[0],anchor[1],origin_z+anchor[2]);
      dJointSetHingeAxis(motor.joint,axis[0],axis[1],axis[2]);
      motor.p=p.get<double>("p");motor.i=p.get<double>("i");motor.imax=p.get<double>("i_clamp_nm");
      motor.effort=p.get<double>("effort_nm");motor.velocity=p.get<double>("velocity_radps");
      motor.damping=p.get<double>("damping_nms_per_rad");motor.antiwindup=p.get<bool>("antiwindup");
      if(motor.i<0 || motor.p<0 || motor.effort<=0 || motor.velocity<=0 || motor.imax<0 || motor.damping<0)
        throw std::runtime_error("invalid PI or joint limits");
      ++mi;
    }
    if(mi!=4) throw std::runtime_error("four ordered motors required: FR FL RL RR");
    ptree ground_config;ground_config.add_child("surface",cfg.get_child("ground"));
    collisions.emplace_back(-1,ground_config);auto& ground=collisions.back();
    ground.geom=dCreatePlane(space,0,0,1,0);dGeomSetData(ground.geom,&ground);
  }
  ~Plant(){dJointGroupDestroy(contacts);dSpaceDestroy(space);dWorldDestroy(world);}
  static void near(void* data,dGeomID first,dGeomID second) {
    static_cast<Plant*>(data)->collide(first,second);
  }
  void collide(dGeomID first,dGeomID second) {
    auto* a=static_cast<Collision*>(dGeomGetData(first));
    auto* b=static_cast<Collision*>(dGeomGetData(second));
    if(!a || !b || (a->body_index>=0 && b->body_index>=0)) return; // self_collide=false
    if(a->body_index<0){std::swap(a,b);std::swap(first,second);}
    dContact contact[64]{};
    const int generated=dCollide(first,second,64,&contact[0].geom,sizeof(dContact));
    const int count=std::min(generated,std::min(global_max_contacts,std::min(a->surface.max_contacts,b->surface.max_contacts)));
    if(count==0) return;
    std::vector<int> indices(generated);for(int k=0;k<generated;++k) indices[k]=k;
    if(generated>count) {
      // Preserve Gazebo's actual last-slot deepest-contact reduction.
      double depth=contact[count-1].geom.depth;
      for(int k=count;k<generated;++k)if(contact[k].geom.depth>depth){depth=contact[k].geom.depth;indices[count-1]=k;}
    }
    for(int k=0;k<count;++k) {
      auto& c=contact[indices[k]];
      c.surface.mode=dContactBounce|dContactMu2|dContactSoftERP|dContactSoftCFM|
          dContactApprox1|dContactApprox3|dContactSlip1|dContactSlip2;
      c.surface.mu=std::min(a->surface.mu,b->surface.mu);
      c.surface.mu2=std::min(a->surface.mu2,b->surface.mu2);
      c.surface.bounce=0;c.surface.bounce_vel=100000;
      const double kp=1./(1./a->surface.kp+1./b->surface.kp),kd=a->surface.kd+b->surface.kd;
      c.surface.soft_erp=h*kp/(h*kp+kd);c.surface.soft_cfm=1./(h*kp+kd);
      c.surface.slip1=(a->surface.slip1+b->surface.slip1)*count;
      c.surface.slip2=(a->surface.slip2+b->surface.slip2)*count;
      const auto& fd=a->surface.fdir;
      if(fd[0]!=0 || fd[1]!=0 || fd[2]!=0) {
        const auto rot=dGeomGetRotation(first);c.surface.mode|=dContactFDir1;
        for(int row=0;row<3;++row)c.fdir1[row]=rot[4*row]*fd[0]+rot[4*row+1]*fd[1]+rot[4*row+2]*fd[2];
      }
      dJointID joint=dJointCreateContact(world,contacts,&c);
      dJointAttach(joint,dGeomGetBody(first),nullptr);
      receipts.emplace_back();auto& receipt=receipts.back();receipt.body=a->body_index;
      for(int axis=0;axis<3;++axis)receipt.point[axis]=c.geom.pos[axis];
      dJointSetFeedback(joint,&receipt.feedback);
    }
  }
  void step(const std::array<double,4>& targets) {
    dJointGroupEmpty(contacts);receipts.clear();
    for(int k=0;k<4;++k) {
      auto& m=motors[k];m.target=clip(targets[k],m.velocity);
      const double omega=dJointGetHingeAngleRate(m.joint),error=m.target-omega;
      m.integral+=h*error;
      if(m.antiwindup && m.i>0)m.integral=clip(m.integral,m.imax/m.i);
      const double integral_torque=clip(m.i*m.integral,m.imax);
      m.torque=clip(m.p*error+integral_torque,m.effort);
      // ODEJoint::ApplyExplicitStiffnessDamping is separate from actuator effort.
      dJointAddHingeTorque(m.joint,m.torque-m.damping*omega);
    }
    dSpaceCollide(space,this,&Plant::near);
    dWorldQuickStep(world,h);
  }
  std::array<double,4> target(const Sample& command) const {
    if(wheel_mode)return command.value;
    const double v=clip(command.value[0],vmax),w=clip(command.value[1],wmax);
    const double left=gain*(v-angular_gain*w*span*.5)/radius;
    const double right=gain*(v+angular_gain*w*span*.5)/radius;
    return {{right,left,left,right}};
  }
  void write(std::ostream& out,double time) const {
    dVector3 pos,vel,body_vel;
    dBodyGetRelPointPos(bodies[0],-base_com[0],-base_com[1],-base_com[2],pos);
    dBodyGetRelPointVel(bodies[0],-base_com[0],-base_com[1],-base_com[2],vel);
    dBodyVectorFromWorld(bodies[0],vel[0],vel[1],vel[2],body_vel);
    const auto rot=dBodyGetRotation(bodies[0]);
    const double yaw=std::atan2(rot[4],rot[0]);
    dVector3 angular;const auto w=dBodyGetAngularVel(bodies[0]);dBodyVectorFromWorld(bodies[0],w[0],w[1],w[2],angular);
    if(!std::isfinite(pos[0]) || !std::isfinite(body_vel[0]) || !std::isfinite(angular[2]))throw std::runtime_error("nonfinite plant state");
    out<<time<<','<<pos[0]<<','<<pos[1]<<','<<pos[2]<<','<<yaw<<','<<body_vel[0]<<','<<body_vel[1]<<','<<angular[2]
       <<','<<std::atan2(rot[9],rot[10])<<','<<std::asin(clip(-rot[8],1.));
    for(const auto& motor:motors)out<<','<<motor.target<<','<<dJointGetHingeAngleRate(motor.joint)<<','<<motor.torque<<','<<motor.integral;
    std::array<std::array<double,3>,5> forces{};
    std::array<double,5> slip_x{},slip_y{},loads{};
    for(const auto& receipt:receipts) {
      const int index=receipt.body;
      for(int k=0;k<3;++k)forces[index][k]+=receipt.feedback.f1[k];
      const double load=std::max(0.,static_cast<double>(receipt.feedback.f1[2]));
      dVector3 point_vel,local;dBodyGetPointVel(bodies[index],receipt.point[0],receipt.point[1],receipt.point[2],point_vel);
      dBodyVectorFromWorld(bodies[0],point_vel[0],point_vel[1],point_vel[2],local);
      slip_x[index]+=load*local[0];slip_y[index]+=load*local[1];loads[index]+=load;
    }
    const double missing=std::numeric_limits<double>::quiet_NaN();
    for(int index=1;index<5;++index)out<<','<<forces[index][0]<<','<<forces[index][1]<<','<<forces[index][2]
      <<','<<(loads[index]>0?slip_x[index]/loads[index]:missing)<<','<<(loads[index]>0?slip_y[index]/loads[index]:missing);
    out<<','<<receipts.size()<<'\n';
  }
};
}

int main(int argc,char**argv) {
  try {
    if(argc!=4){std::cerr<<"usage: scout_native config.json input.csv output.csv\n";return 2;}
    ptree cfg;boost::property_tree::read_json(argv[1],cfg);
    const bool wheel=cfg.get<std::string>("input_mode")=="wheel_targets";
    const auto command=inputs(argv[2],wheel);
    if(!dInitODE2(0))throw std::runtime_error("ODE initialization failed");
    dRandSetSeed(93);
    {
      Plant plant(cfg);
      std::array<double,4> held{};
      const double settle=cfg.get<double>("settle_s"),tail=cfg.get<double>("tail_s");
      if(settle<0 || tail<0)throw std::runtime_error("invalid fixed run window");
      for(int k=0;k<static_cast<int>(std::ceil(settle/plant.h));++k)plant.step(held);
      std::ofstream out(argv[3]);if(!out)throw std::runtime_error("cannot create output");out<<std::setprecision(17);
      out<<"time_s,x_m,y_m,z_m,yaw_rad,v_parallel_mps,v_perp_mps,wz_radps,roll_rad,pitch_rad";
      for(const auto*name:{"fr","fl","rl","rr"})out<<','<<name<<"_target_radps,"<<name<<"_omega_radps,"<<name<<"_torque_nm,"<<name<<"_integral_rad";
      for(const auto*name:{"fr","fl","rl","rr"})out<<','<<name<<"_fx_n,"<<name<<"_fy_n,"<<name<<"_fz_n,"<<name<<"_contact_slip_x_mps,"<<name<<"_contact_slip_y_mps";
      out<<",contact_points\n";
      std::size_t next=0;
      const int steps=static_cast<int>(std::ceil((command.back().t+tail)/plant.h));
      for(int k=0;k<steps;++k) {
        const double time=k*plant.h;
        while(next<command.size() && command[next].t+(wheel?0.:plant.delay)<=time+1.e-12)held=plant.target(command[next++]);
        plant.step(held);plant.write(out,(k+1)*plant.h);
      }
      std::cout<<"native ODE reconstruction wrote "<<steps<<" physics steps; not physical acceptance\n";
    }
    dCloseODE();return 0;
  } catch(const std::exception&error){std::cerr<<"scout_native: "<<error.what()<<'\n';return 2;}
}
