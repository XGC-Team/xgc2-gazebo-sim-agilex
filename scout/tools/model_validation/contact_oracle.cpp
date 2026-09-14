// A native test of Gazebo 11.15.1's ODE contact implementation.
// This is an isolated sliding-box oracle, NOT a fitted Scout plant.
#include <gazebo/ode/ode.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
constexpr double kStep = 0.001;
struct World {
  dWorldID world = dWorldCreate();
  dSpaceID space = dSimpleSpaceCreate(nullptr);
  dJointGroupID contacts = dJointGroupCreate(0);
  dBodyID body = nullptr;
  dGeomID box = nullptr;
  dGeomID ground = nullptr;
  explicit World(Friction_Model model) {
    dWorldSetGravity(world, 0, 0, -9.81);
    dWorldSetAutoDisableFlag(world, 0);
    dWorldSetQuickStepNumIterations(world, 100);
    dWorldSetQuickStepW(world, 1.0);
    dWorldSetQuickStepFrictionModel(world, model);
    body = dBodyCreate(world);
    dMass mass;
    dMassSetBoxTotal(&mass, 1.0, 0.2, 0.2, 0.2);
    dBodySetMass(body, &mass);
    dBodySetPosition(body, 0, 0, 0.1);
    box = dCreateBox(space, 0.2, 0.2, 0.2);
    dGeomSetBody(box, body);
    ground = dCreatePlane(space, 0, 0, 1, 0);
  }
  ~World() {
    dJointGroupDestroy(contacts);
    dSpaceDestroy(space);
    dWorldDestroy(world);
  }
  void step() {
    dContact contact[16]{};
    const int count = dCollide(box, ground, 16, &contact[0].geom, sizeof(dContact));
    for (int i = 0; i < count; ++i) {
      auto &surface = contact[i].surface;
      surface.mode = dContactMu2 | dContactApprox1 | dContactSoftERP |
                     dContactSoftCFM | dContactFDir1 | dContactSlip1 | dContactSlip2;
      surface.mu = 0.6;
      surface.mu2 = 0.6;
      surface.soft_erp = 0.2;
      surface.soft_cfm = 1.e-8;
      // ODEPhysics::Collide scales per-collision slip by contact count.
      surface.slip1 = 0.001 * count;
      surface.slip2 = 0.001 * count;
      contact[i].fdir1[0] = 1.0;
      auto joint = dJointCreateContact(world, contacts, &contact[i]);
      dJointAttach(joint, body, nullptr);
    }
    dWorldQuickStep(world, kStep);
    dJointGroupEmpty(contacts);
  }
};

double run(Friction_Model model) {
  World test(model);
  for (int i = 0; i < 300; ++i) test.step();
  dBodySetLinearVel(test.body, 1.0, 1.0, 0.0);
  for (int i = 0; i < 50; ++i) test.step();
  const auto velocity = dBodyGetLinearVel(test.body);
  return std::hypot(1.0 - velocity[0], 1.0 - velocity[1]) / (50 * kStep * 9.81);
}
}  // namespace

int main() {
  if (!dInitODE2(0)) return 2;
  const double pyramid = run(pyramid_friction);
  const double cone = run(cone_friction);
  dCloseODE();
  std::cout << std::setprecision(12)
            << "{\"pyramid_deceleration_over_g\":" << pyramid
            << ",\"cone_deceleration_over_g\":" << cone
            << ",\"mu\":0.6,\"step_s\":0.001,\"scope\":\"native ODE contact, not Scout acceptance\"}\n";
  if (!std::isfinite(pyramid) || !std::isfinite(cone) ||
      pyramid < 0.75 || pyramid > 0.95 || cone < 0.50 || cone > 0.70)
    return 1;
  return 0;
}
