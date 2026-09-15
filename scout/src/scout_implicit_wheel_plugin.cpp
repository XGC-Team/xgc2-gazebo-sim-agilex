#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/transport/transport.hh>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <xgc2_math/control/delayed_planar_velocity.hpp>
#include <xgc2_math/control/implicit_wheel_contact.hpp>
#include "xgc_chassis_hold/udp.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace scout_gazebo {

// Experimental flat-ground force model. ODE owns normal/obstacle contact and
// rigid-body integration; this plugin owns ALL wheel motor/tangential forces.
// Predicted velocities from the implicit math kernel are NEVER written to Gazebo.
class ImplicitWheelPlugin final : public gazebo::ModelPlugin {
    using Vec = ignition::math::Vector3d;
    struct Patch { double normal{0}; Vec point{0,0,0}; double time{-1}; std::string support; };
public:
    ~ImplicitWheelPlugin() override {
        begin_.reset(); end_.reset();
        stopping_.store(true); queue_.disable();
        if (thread_.joinable()) thread_.join();
        command_sub_.shutdown();
        if (gate_) xgc_chassis_hold::Hub::instance().remove(gate_.get());
        contact_sub_.reset();
        if (manager_ && !filter_.empty()) manager_->RemoveFilter(filter_);
        if (transport_) transport_->Fini();
    }
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
        if (!ros::isInitialized()) throw std::runtime_error("implicit wheels require gazebo_ros_api_plugin");
        model_=model; world_=model->GetWorld(); base_=model->GetLink("base_link");
        if (!base_ || world_->Physics()->GetType()!="ode")
            throw std::runtime_error("implicit wheels require base_link and Gazebo Classic ODE");
        // Refuse conflicting actuators, including an accidentally reused URDF.
        auto description=model->GetSDF();
        if (description->HasElement("plugin")) for(auto e=description->GetElement("plugin");e;e=e->GetNextElement("plugin")) {
            const auto file=e->Get<std::string>("filename");
            if(file=="libgazebo_ros_control.so" || file=="libscout_unicycle_plugin.so")
                throw std::runtime_error("implicit wheels cannot share wheel or chassis actuation");
        }
        const auto ns=sdf->Get<std::string>("robotNamespace");
        node_.reset(new ros::NodeHandle(ns)); node_->setCallbackQueue(&queue_);
        ros::NodeHandle config(*node_,"implicit_wheel");
        double stiffness, friction, length, delay;
        config.param("stiffness_n_m",stiffness,12000.0);
        config.param("friction",friction,.6);
        config.param("renewal_length_m",length,.04);
        config.param("motor_p",parameters_.motor_p_nm_s,1.8);
        config.param("motor_i",parameters_.motor_i_nm,10.0);
        config.param("effort_limit_nm",parameters_.motor_limit_nm,6.0);
        config.param("passive_drag_nm_s",parameters_.passive_drag_nm_s,0.0);
        config.param("command_delay_s",delay,.005);
        config.param("max_linear_speed",max_v_,1.5);
        config.param("max_angular_speed",max_w_,.5235);
        config.param("wheel_velocity_limit",max_wheel_,26.0);
        config.param("allocation_gain",gain_,1.0);
        config.param("yaw_calibration",yaw_gain_,1.0);
        parameters_.stiffness_n_m.fill(stiffness);parameters_.friction.fill(friction);
        parameters_.renewal_length_m.fill(length);
        for(double x:{max_v_,max_w_,max_wheel_,gain_,yaw_gain_})
            xgc2_math::implicit_wheel_detail::positive(x);
        delayed_.reset(new xgc2_math::DelayedPlanarVelocity({delay,0,0})); // NO extra first-order lag
        last_time_=world_->SimTime().Double();delayed_->reset(last_time_);
        const std::array<std::string,4> names{{"front_left_wheel","front_right_wheel","rear_left_wheel","rear_right_wheel"}};
        std::vector<std::string> collisions;
        for(std::size_t i=0;i<4;++i) {
            joints_[i]=model->GetJoint(names[i]);wheels_[i]=model->GetLink(names[i]+"_link");
            if(!joints_[i] || !wheels_[i] || wheels_[i]->GetCollisions().size()!=1)
                throw std::runtime_error("implicit wheels require four single-collision wheel links");
            collisions_[i]=wheels_[i]->GetCollisions().front();
            collisions.push_back(collisions_[i]->GetScopedName());
            const auto cylinder=boost::dynamic_pointer_cast<gazebo::physics::CylinderShape>(collisions_[i]->GetShape());
            if(!cylinder)throw std::runtime_error("implicit wheels require cylindrical wheel collision shapes");
            nominal_radius_[i]=cylinder->GetRadius();
            xgc2_math::implicit_wheel_detail::positive(nominal_radius_[i]);
            // Local surface only; no global friction/solver changes or mg/4 loads.
            auto friction_pyramid=collisions_[i]->GetSurface()->FrictionPyramid();
            if(!friction_pyramid) throw std::runtime_error("wheel friction surface unavailable");
            friction_pyramid->SetMuPrimary(0);friction_pyramid->SetMuSecondary(0);friction_pyramid->SetMuTorsion(0);
            joints_[i]->SetDamping(0,0); // drag, if requested, is handled once in math
        }
        // Validate constitutive parameters before registering any command source.
        auto probe=xgc2_math::ImplicitWheelInput{};
        (void)xgc2_math::implicitWheelContactStep(probe,{},parameters_);
        joint_pub_=node_->advertise<sensor_msgs::JointState>("joint_states",2);
        report_pub_=node_->advertise<std_msgs::Float64MultiArray>("simulation/implicit_wheels",2);
        fault_pub_=node_->advertise<std_msgs::String>("simulation/dynamics_fault",1,true);
        mode_pub_=node_->advertise<std_msgs::String>("simulation/dynamics_mode",1,true);
        backend_pub_=node_->advertise<std_msgs::String>("simulation/wheel_physics_backend",1,true);
        std_msgs::String label;label.data="wheel_physics";mode_pub_.publish(label);
        label.data="implicit_brush";backend_pub_.publish(label);label.data="";fault_pub_.publish(label);
        manager_=world_->Physics()->GetContactManager();
        filter_="scout_implicit_"+model->GetName();
        const auto topic=manager_->CreateFilter(filter_,collisions);
        transport_.reset(new gazebo::transport::Node);transport_->Init(world_->Name());
        // Subscription requests per-wheel feedback; physics data is read ONLY
        // at UpdateEnd, not consumed from asynchronous message callbacks.
        contact_sub_=transport_->Subscribe(topic,&ImplicitWheelPlugin::ContactsKeepalive,this);
        gate_.reset(new xgc_chassis_hold::Gate(xgc_chassis_hold::lastPath(ns)));
        gate_->setZeroThunk(&ImplicitWheelPlugin::HoldThunk,this);
        xgc_chassis_hold::Hub::instance().add(gate_.get());
        command_sub_=node_->subscribe("cmd_vel",10,&ImplicitWheelPlugin::Command,this);
        begin_=gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ImplicitWheelPlugin::Update,this,std::placeholders::_1));
        end_=gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&ImplicitWheelPlugin::Collect,this));
        thread_=std::thread([this]{while(!stopping_.load() && node_->ok())queue_.callAvailable(ros::WallDuration(.01));});
        ROS_WARN("Scout implicit_brush is an uncalibrated experimental force backend; no field-accuracy claim");
    }
    void Reset() override {
        if(!gate_) return;
        gate_->withCommand([this](bool){
            memory_={};patches_={};last_time_=world_->SimTime().Double();delayed_->reset(last_time_);fault_=false;
            std_msgs::String clear;fault_pub_.publish(clear);
        });
    }
private:
    void ContactsKeepalive(const boost::shared_ptr<const gazebo::msgs::Contacts>&) {}
    static void HoldThunk(void* self) {
        auto* p=static_cast<ImplicitWheelPlugin*>(self);
        p->delayed_->reset(p->last_time_);p->memory_.drive_error_rad.fill(0);
        // No physics API on UDP thread. Existing contact elasticity is preserved.
    }
    void Command(const geometry_msgs::Twist::ConstPtr& msg) {
        if(!std::isfinite(msg->linear.x)||!std::isfinite(msg->angular.z))return;
        gate_->withCommand([&](bool held){
            if(held||fault_)return;
            try{delayed_->command(last_time_,{std::clamp(msg->linear.x,-max_v_,max_v_),
                                            std::clamp(msg->angular.z,-max_w_,max_w_)});}
            catch(const std::exception& e){Fault(e.what());}
        });
    }
    void Fault(const std::string& why) {
        fault_=true;memory_={};delayed_->reset(last_time_);
        std_msgs::String message;message.data=why;fault_pub_.publish(message);
        ROS_ERROR_STREAM("Scout implicit wheel backend latched fault: "<<why);
    }
    void Collect() {
        if(!gate_)return;
        gate_->withCommand([&](bool){
            patches_={};
            const double now=world_->SimTime().Double();
            for(unsigned k=0;k<manager_->GetContactCount();++k) {
                const auto* contact=manager_->GetContact(k);if(!contact)continue;
                for(std::size_t i=0;i<4;++i){
                    const bool first=contact->collision1==collisions_[i].get();
                    if(!first && contact->collision2!=collisions_[i].get())continue;
                    auto* other=first?contact->collision2:contact->collision1;
                    for(int j=0;j<contact->count;++j){
                        // Only near-horizontal static support belongs to this
                        // reduced tire model. Obstacle normals remain ODE's job.
                        if(std::abs(contact->normals[j].Z())<.999)continue;
                        if(!other || !other->GetLink()->GetModel()->IsStatic()){
                            if(!fault_)Fault("moving support is outside implicit_brush validity");continue;
                        }
                        const auto force=first?contact->wrench[j].body1Force:contact->wrench[j].body2Force;
                        const double n=std::max(0.0,force.Z());
                        if(!std::isfinite(n)||!contact->positions[j].IsFinite()){
                            if(!fault_)Fault("nonfinite contact feedback");continue;
                        }
                        const auto support=other->GetScopedName();
                        if(patches_[i].support.empty())patches_[i].support=support;
                        else if(patches_[i].support!=support)patches_[i].support="multiple-static-supports";
                        patches_[i].normal+=n;patches_[i].point+=n*contact->positions[j];patches_[i].time=now;
                    }
                }
            }
            for(auto& patch:patches_)if(patch.normal>0)patch.point/=patch.normal;
        });
    }
    void Update(const gazebo::common::UpdateInfo& info) {
        gate_->withCommand([&](bool held){
            const double now=info.simTime.Double();
            if(now<last_time_){memory_={};patches_={};delayed_->reset(now);fault_=false;}
            last_time_=now;
            if(fault_){for(auto& joint:joints_)joint->SetForce(0,0);return;}
            try {
                const double h=world_->Physics()->GetMaxStepSize();
                if(!(h>0 && h<=.02))throw std::runtime_error("unsupported physics step (0,20 ms]");
                const auto rotation=base_->WorldPose().Rot();
                if(std::abs(rotation.Roll())>.0873 || std::abs(rotation.Pitch())>.0873)
                    throw std::runtime_error("roll/pitch exceeds planar backend validity (5 degrees)");
                const auto gravity=world_->Gravity();
                if(std::hypot(gravity.X(),gravity.Y())>1e-6 || gravity.Z()>=0)
                    throw std::runtime_error("implicit_brush requires vertical downward gravity");
                Vec com(0,0,0),velocity(0,0,0);double mass=0,iz=0;
                for(const auto& link:model_->GetLinks()){
                    const double m=link->GetInertial()->Mass();mass+=m;
                    com+=m*link->WorldCoGPose().Pos();velocity+=m*link->WorldCoGLinearVel();
                }
                xgc2_math::implicit_wheel_detail::positive(mass);com/=mass;velocity/=mass;
                for(const auto& link:model_->GetLinks()){
                    const auto r=link->WorldCoGPose().Pos()-com;
                    iz+=link->WorldInertiaMatrix()(2,2)+link->GetInertial()->Mass()*(r.X()*r.X()+r.Y()*r.Y());
                }
                parameters_.mass_kg=mass;parameters_.yaw_inertia_kg_m2=iz;
                xgc2_math::ImplicitWheelInput input;input.dt_s=h;
                input.velocity[0]=velocity.X();input.velocity[1]=velocity.Y();input.velocity[2]=base_->WorldAngularVel().Z();
                auto command=delayed_->advance(now);if(held)command={};
                const Vec forward=rotation.RotateVector(Vec(1,0,0));
                std::array<Vec,4> point{};std::array<double,4> sign{};
                for(std::size_t i=0;i<4;++i){
                    const auto axis=joints_[i]->GlobalAxis(0).Normalized();
                    auto rolling=axis.Cross(Vec(0,0,1));
                    if(rolling.Length()<.99)throw std::runtime_error("wheel axle is not horizontal");
                    rolling.Normalize();sign[i]=rolling.Dot(forward)>=0?1:-1;rolling*=sign[i];
                    const auto anchor=joints_[i]->Anchor(0);
                    const auto center=collisions_[i]->WorldPose().Pos();
                    point[i]=center-Vec(0,0,nominal_radius_[i]); // only a lever-arm reference when unloaded
                    auto patch=patches_[i];
                    if(support_[i]!=patch.support){memory_.tread_m[i]={};support_[i]=patch.support;}
                    // One completed-step normal feedback; reject stale records.
                    if(patch.normal>0 && now>=patch.time-1e-9 && now-patch.time<=1.5*h){
                        // A separating wheel cannot reuse last step's normal load.
                        const double bottom=collisions_[i]->BoundingBox().Min().Z();
                        if(bottom<=patch.point.Z()+.002){input.normal_force_n[i]=patch.normal;point[i]=patch.point;}
                    }
                    const auto spin_arm=sign[i]*axis.Cross(point[i]-anchor);
                    const double radius=std::hypot(spin_arm.X(),spin_arm.Y());
                    if(radius<.5*nominal_radius_[i]||radius>1.5*nominal_radius_[i])
                        throw std::runtime_error("contact lever arm is outside wheel geometry");
                    input.radius_m[i]=radius;
                    const auto offset=point[i]-com;
                    input.contact_offset_world_m[i]={offset.X(),offset.Y()};
                    // Exact planar force-to-spin moment arm at the chosen contact.
                    input.rolling_direction_world[i]={-spin_arm.X()/radius,-spin_arm.Y()/radius};
                    input.velocity[3+i]=sign[i]*joints_[i]->GetVelocity(0);
                    const auto cog_offset=wheels_[i]->WorldCoGPose().Pos()-anchor;
                    parameters_.wheel_inertia[i]=axis.Dot(wheels_[i]->WorldInertiaMatrix()*axis)
                        +wheels_[i]->GetInertial()->Mass()*cog_offset.Cross(axis).SquaredLength();
                    const auto body_point=rotation.RotateVectorReverse(center-base_->WorldPose().Pos());
                    input.target_rad_s[i]=std::clamp(gain_*(command.linear_m_s-yaw_gain_*command.yaw_rad_s*body_point.Y())/nominal_radius_[i],
                                                     -max_wheel_,max_wheel_);
                }
                const auto result=xgc2_math::implicitWheelContactStep(input,memory_,parameters_);
                // All values are accepted before ANY force write. Apply traction
                // at the actual contact centroid to the WHEEL, so its -R*Fx
                // reaction is generated once by rigid-body mechanics, not twice.
                for(std::size_t i=0;i<4;++i){
                    const auto f=result.tire_force_world_n[i];
                    wheels_[i]->AddForceAtWorldPosition(Vec(f[0],f[1],0),point[i]);
                    joints_[i]->SetForce(0,sign[i]*result.joint_torque_nm[i]);
                }
                memory_=result.memory;
                Publish(input,result,sign);
            }catch(const std::exception& e){Fault(e.what());for(auto& joint:joints_)joint->SetForce(0,0);}
        });
    }
    void Publish(const xgc2_math::ImplicitWheelInput& input,const xgc2_math::ImplicitWheelResult& result,
                 const std::array<double,4>& sign) {
        // Measurement headers follow the existing ROS clock contract. Diagnostic
        // first column carries physics time explicitly (including hybrid runs).
        sensor_msgs::JointState state;state.header.stamp=ros::Time::now();
        std_msgs::Float64MultiArray report;
        report.layout.dim.resize(2);report.layout.dim[0].label="wheel_FL_FR_RL_RR";
        report.layout.dim[0].size=4;report.layout.dim[0].stride=56;
        report.layout.dim[1].label="sim_time,target,measured,normal,Fx,Fy,tau,predicted_slip_x,predicted_slip_y,tread_x,tread_y,drive_error,kkt,sweeps";
        report.layout.dim[1].size=14;report.layout.dim[1].stride=14;
        for(std::size_t i=0;i<4;++i){
            state.name.push_back(joints_[i]->GetName());state.position.push_back(joints_[i]->Position(0));
            state.velocity.push_back(joints_[i]->GetVelocity(0));state.effort.push_back(sign[i]*result.joint_torque_nm[i]);
            const double row[]={last_time_,input.target_rad_s[i],input.velocity[3+i],input.normal_force_n[i],
                result.tire_force_world_n[i][0],result.tire_force_world_n[i][1],result.motor_torque_nm[i],
                result.predicted_surface_slip_m_s[i][0],result.predicted_surface_slip_m_s[i][1],
                result.memory.tread_m[i][0],result.memory.tread_m[i][1],result.memory.drive_error_rad[i],
                result.kkt_residual,static_cast<double>(result.sweeps)};
            report.data.insert(report.data.end(),std::begin(row),std::end(row));
        }
        joint_pub_.publish(state);report_pub_.publish(report);
    }
    gazebo::physics::ModelPtr model_;gazebo::physics::WorldPtr world_;gazebo::physics::LinkPtr base_;
    std::array<gazebo::physics::JointPtr,4> joints_;
    std::array<gazebo::physics::LinkPtr,4> wheels_;
    std::array<gazebo::physics::CollisionPtr,4> collisions_;
    gazebo::physics::ContactManager* manager_{nullptr};
    gazebo::event::ConnectionPtr begin_,end_;
    gazebo::transport::NodePtr transport_;gazebo::transport::SubscriberPtr contact_sub_;std::string filter_;
    std::unique_ptr<ros::NodeHandle> node_;ros::CallbackQueue queue_;std::thread thread_;std::atomic<bool> stopping_{false};
    ros::Subscriber command_sub_;ros::Publisher joint_pub_,report_pub_,fault_pub_,mode_pub_,backend_pub_;
    std::unique_ptr<xgc_chassis_hold::Gate> gate_;
    std::unique_ptr<xgc2_math::DelayedPlanarVelocity> delayed_;
    xgc2_math::ImplicitWheelParameters parameters_;xgc2_math::ImplicitWheelMemory memory_;
    std::array<Patch,4> patches_{};std::array<double,4> nominal_radius_{};std::array<std::string,4> support_{};
    double last_time_{0},max_v_{1.5},max_w_{.5235},max_wheel_{26},gain_{1},yaw_gain_{1};bool fault_{false};
};
GZ_REGISTER_MODEL_PLUGIN(ImplicitWheelPlugin)
} // namespace scout_gazebo
