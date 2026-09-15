#!/usr/bin/env python3
"""Probe-free adapter gate, not field replay or motor-memory acceptance.

Run only through implicit_contact_feedback.test in an isolated Noetic/Gazebo
master. The synthetic input is 2 s rest, one 0.3 m/s command for 8 s, then one
Stop for 10 s. This is NOT the harness's 520-command field-bag prefix.
"""
from collections import deque
import json
import math
import os
from pathlib import Path
import threading
import time
import unittest

try:
    import rospy
    import rostest
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float64MultiArray, String
    from std_srvs.srv import Empty
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import (GetModelState, SetModelState, GetModelProperties,
                                 GetLinkProperties, GetPhysicsProperties,
                                 SetPhysicsProperties)
    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False


@unittest.skipUnless(_HAS_ROS and __name__ == '__main__', 'requires isolated rostest launch')
class ContactRuntime(unittest.TestCase):
    model = 'brush_contact_test'

    def setUp(self):
        self.lock = threading.Lock()
        self.wheels = deque(maxlen=30000)
        self.contacts = deque(maxlen=30000)
        self.fault = ''
        self.backend = ''
        prefix = '/' + self.model
        self.subs = [
            rospy.Subscriber(prefix + '/simulation/implicit_wheels', Float64MultiArray,
                             lambda m: self.record(m, 14, self.wheels)),
            rospy.Subscriber(prefix + '/simulation/implicit_contacts', Float64MultiArray,
                             lambda m: self.record(m, 23, self.contacts)),
            rospy.Subscriber(prefix + '/simulation/dynamics_fault', String,
                             lambda m: setattr(self, 'fault', m.data)),
            rospy.Subscriber(prefix + '/simulation/wheel_physics_backend', String,
                             lambda m: setattr(self, 'backend', m.data)),
        ]
        self.pub = rospy.Publisher(prefix + '/cmd_vel', Twist, queue_size=1)
        self.state = self.service('get_model_state', GetModelState)
        self.pause = self.service('pause_physics', Empty)
        self.unpause = self.service('unpause_physics', Empty)
        self.reset = self.service('reset_simulation', Empty)
        self.set_state = self.service('set_model_state', SetModelState)
        get_physics = self.service('get_physics_properties', GetPhysicsProperties)
        set_physics = self.service('set_physics_properties', SetPhysicsProperties)
        physics = get_physics()
        self.assertTrue(physics.success, physics.status_message)
        self.step = float(rospy.get_param('~step_size', .004))
        result = set_physics(self.step, 1.0 / self.step, physics.gravity, physics.ode_config)
        self.assertTrue(result.success, result.status_message)
        props = self.service('get_model_properties', GetModelProperties)
        link_props = self.service('get_link_properties', GetLinkProperties)
        self.wait(lambda: bool(self.latest(self.wheels)) and bool(self.latest(self.contacts))
                  and self.pub.get_num_connections() > 0, 40)
        model = props(self.model)
        self.assertTrue(model.success, model.status_message)
        mass = 0.0
        for name in model.body_names:
            link = link_props(name if '::' in name else self.model + '::' + name)
            self.assertTrue(link.success, link.status_message)
            mass += link.mass
        self.weight = mass * math.sqrt(sum(getattr(physics.gravity, k)**2 for k in 'xyz'))
        self.assertGreater(self.weight, 0)
        self.assertEqual(self.backend, 'implicit_brush')

    def service(self, name, kind):
        name = '/gazebo/' + name
        rospy.wait_for_service(name, timeout=40)
        return rospy.ServiceProxy(name, kind)

    def record(self, message, columns, target):
        if len(message.data) != 4 * columns:
            self.fault = 'unexpected diagnostic shape'
            return
        rows = [list(message.data[i * columns:(i + 1) * columns]) for i in range(4)]
        with self.lock:
            target.append(rows)

    def latest(self, history):
        with self.lock:
            return history[-1] if history else None

    def samples(self, history, since):
        with self.lock:
            return [rows for rows in history if rows[0][0] >= since]

    def wait(self, predicate, seconds=40):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.fault:
                self.fail(self.fault)
            if predicate():
                return
            time.sleep(.005)
        self.fail('timed out waiting for fresh physical feedback')

    def advance(self, seconds):
        target = self.latest(self.contacts)[0][0] + seconds
        self.wait(lambda: self.latest(self.contacts)[0][0] >= target, max(40, 4 * seconds))

    def command(self, velocity, yaw=0.0):
        command = Twist()
        command.linear.x = velocity
        command.angular.z = yaw
        self.pub.publish(command)

    def physical_state(self):
        result = self.state(self.model, 'world')
        self.assertTrue(result.success, result.status_message)
        return result

    def test_probe_free_support_traction_stop_and_lifecycle(self):
        self.command(0)
        start = self.latest(self.contacts)[0][0]
        self.advance(2)
        stationary = self.samples(self.wheels, start + 1)
        self.assertGreater(len(stationary), 20)
        means = [sum(rows[i][3] for rows in stationary) / len(stationary) for i in range(4)]
        for load in means:
            self.assertGreater(load, 1.0)  # No assumed mg/4 or exact equal loading.
        self.assertLess(abs(sum(means) - self.weight), .15 * self.weight)
        for rows in self.samples(self.contacts, start + 1):
            for row in rows:
                self.assertTrue(all(math.isfinite(value) for value in row))
                self.assertEqual(row[22], 1)  # Plugin's own filter remains alive.
                self.assertEqual(row[9] + row[10] + row[11], 0)
                if row[8] > 0:
                    self.assertAlmostEqual(row[0], row[1], places=8)
                    self.assertAlmostEqual(row[0], row[2], places=8)
                    self.assertAlmostEqual(row[0], row[3], places=8)

        initial = self.physical_state()
        drive_start = self.latest(self.contacts)[0][0]
        self.command(.3)  # One message; no body-velocity feedback controller.
        self.advance(8)
        final = self.physical_state()
        displacement = math.hypot(final.pose.position.x - initial.pose.position.x,
                                  final.pose.position.y - initial.pose.position.y)
        self.assertGreater(displacement, .2, 'spinning wheels alone are not traction')
        driving = self.samples(self.wheels, drive_start)
        peak_force = max(math.hypot(row[4], row[5]) for rows in driving for row in rows)
        self.assertGreater(peak_force, .5)
        for rows in driving:
            for row in rows:
                self.assertTrue(all(math.isfinite(value) for value in row))
                self.assertLessEqual(math.hypot(row[4], row[5]), .6 * row[3] + 1e-5)
                self.assertLessEqual(abs(row[6]), 6.00001)
        self.command(0)  # Exactly one Stop, not a pose/velocity reset.
        self.advance(10)
        stopped = self.physical_state()
        stop_speed = math.hypot(stopped.twist.linear.x, stopped.twist.linear.y)
        self.assertLess(stop_speed, .05)
        self.assertLess(abs(stopped.twist.angular.z), .06)

        # Lifecycle checks are AFTER the synthetic 20 s input. Only this test,
        # never the production plugin, teleports a model to exercise lift-off.
        self.pause()
        before_lift = self.latest(self.contacts)[0][0]
        lift = ModelState()
        lift.model_name = self.model
        lift.reference_frame = 'world'
        lift.pose = stopped.pose
        lift.pose.position.z += .6
        result = self.set_state(lift)
        self.assertTrue(result.success, result.status_message)
        self.unpause()
        self.wait(lambda: self.latest(self.wheels)[0][0] > before_lift + 2 * self.step
                  and all(row[3] == 0 and abs(row[4]) + abs(row[5]) < 1e-9
                          for row in self.latest(self.wheels)), 5)
        self.assertGreater(self.physical_state().pose.position.z, .35)
        pre_reset = self.latest(self.contacts)[0][0]
        self.reset()
        self.wait(lambda: self.latest(self.contacts)[0][0] < pre_reset - 1, 5)
        self.wait(lambda: all(row[3] > 1 for row in self.latest(self.wheels)), 20)

        output = Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros')))
        output.mkdir(parents=True, exist_ok=True)
        (output / 'implicit-contact-summary.json').write_text(json.dumps({
            'input': 'synthetic 2 s rest / 8 s drive / 10 s Stop; not field bag',
            'physics_step_s': self.step, 'extra_probe': False,
            'weight_n': self.weight, 'stationary_wheel_normal_n': means,
            'drive_displacement_m': displacement, 'peak_tire_force_n': peak_force,
            'stop_speed_m_s': stop_speed,
        }, indent=2) + '\n')

    def tearDown(self):
        self.command(0)
        self.unpause()
        for subscriber in self.subs:
            subscriber.unregister()
        self.pub.unregister()


if __name__ == '__main__':
    if not _HAS_ROS:
        raise SystemExit('requires isolated ROS/Gazebo')
    rospy.init_node('implicit_contact_regression')
    rostest.rosrun('gazebo_sim_scout', 'implicit_contact_feedback', ContactRuntime)
