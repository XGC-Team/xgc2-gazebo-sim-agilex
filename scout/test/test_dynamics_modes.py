#!/usr/bin/env python3
"""Source wiring tests. These do not claim a ROS/Gazebo runtime experiment."""
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
ARGS = ('dynamics_mode', 'unicycle_linear_time_constant_s',
        'unicycle_angular_time_constant_s', 'wheel_diagnostics')


def launch(name):
    return ET.parse(PACKAGE / 'launch' / name).getroot()


def fields(element, attribute='value'):
    return {a.get('name'): a.get(attribute) for a in element.findall('./arg')}


class DynamicsModesTest(unittest.TestCase):
    def test_enum_rejects_unknown_values(self):
        expression = fields(launch('spawn_accurate.launch'))['wheel_physics']
        self.assertEqual(expression, "$(eval {'wheel_physics': True, 'unicycle': False}[arg('dynamics_mode')])")
        for mode, expected in [('wheel_physics', True), ('unicycle', False)]:
            self.assertEqual(eval(expression[7:-1], {'__builtins__': {}}, {'arg': lambda _: mode}), expected)
        with self.assertRaises(KeyError):
            eval(expression[7:-1], {'__builtins__': {}}, {'arg': lambda _: 'typo'})

    def test_defaults_are_distinct(self):
        for name, mode in [('spawn_accurate.launch', 'wheel_physics'),
                           ('accurate.launch', 'wheel_physics'),
                           ('multi_accurate.launch', 'wheel_physics'),
                           ('simple.launch', 'unicycle')]:
            defaults = fields(launch(name), 'default')
            self.assertEqual(defaults['dynamics_mode'], mode)
            self.assertEqual(defaults['command_delay_s'], '0.005')
            for key in ARGS[1:3]:
                self.assertEqual(defaults[key], '0.005')

    def test_wheel_controllers_are_exclusive(self):
        root = launch('spawn_accurate.launch')
        wheel_nodes = [node for node in root.findall('node')
                       if node.get('type') in ('spawner', 'scout_skid_steer_controller')]
        self.assertEqual(len(wheel_nodes), 2)
        for node in wheel_nodes:
            self.assertEqual(node.get('if'), '$(arg wheel_physics)')
        for group in root.findall('group'):
            if group.find('rosparam') is not None:
                self.assertEqual(group.get('if'), '$(arg wheel_physics)')

    def test_low_mode_disables_wheel_ground_actuation(self):
        root = launch('spawn_accurate.launch')
        includes = root.findall("include[@file='$(dirname)/mini_description.launch']")
        self.assertEqual(len(includes), 2)
        high, low = includes
        self.assertEqual(high.get('if'), '$(arg wheel_physics)')
        self.assertEqual(low.get('unless'), '$(arg wheel_physics)')
        h, l = fields(high), fields(low)
        self.assertEqual(h['enable_ros_control'], 'true')
        self.assertEqual(l['enable_ros_control'], 'false')
        for key in ('wheel_contact_mu1', 'wheel_contact_mu2', 'wheel_contact_slip1',
                    'wheel_contact_slip2', 'wheel_joint_damping'):
            self.assertEqual(h[key], '$(arg %s)' % key)
            self.assertEqual(l[key], '0.0')
        self.assertEqual(h['urdf_extras'], l['urdf_extras'])
        for key in h.keys() - {'enable_ros_control', 'wheel_contact_mu1', 'wheel_contact_mu2',
                              'wheel_contact_slip1', 'wheel_contact_slip2', 'wheel_joint_damping'}:
            self.assertEqual(h[key], l[key])

    def test_lag_only_in_low_mode(self):
        root = launch('spawn_accurate.launch')
        group = root.find("group[@ns='$(arg ns)/unicycle']")
        self.assertEqual(group.get('unless'), '$(arg wheel_physics)')
        params = {p.get('name'): p.get('value') for p in group.findall('param')}
        self.assertEqual(params['command_delay_s'], '$(arg command_delay_s)')
        self.assertEqual(params['linear_time_constant_s'], '$(arg unicycle_linear_time_constant_s)')
        self.assertEqual(params['angular_time_constant_s'], '$(arg unicycle_angular_time_constant_s)')
        high = root.find("node[@type='scout_skid_steer_controller']")
        self.assertFalse(any('time_constant' in p.get('name') for p in high.findall('param')))

    def test_single_robot_forwarding(self):
        for name in ('accurate.launch', 'simple.launch'):
            include = launch(name).find("include[@file='$(dirname)/spawn_accurate.launch']")
            for key in ARGS + ('command_delay_s',):
                self.assertEqual(fields(include)[key], '$(arg %s)' % key)

    def test_per_robot_selection(self):
        root = launch('multi_accurate.launch')
        defaults = fields(root, 'default')
        includes = root.findall("include[@file='$(dirname)/spawn_accurate.launch']")
        self.assertEqual(len(includes), 2)
        for i, include in enumerate(includes, 1):
            for key in ARGS:
                parameter = 'ugv%d_%s' % (i, key)
                self.assertEqual(defaults[parameter], '$(arg %s)' % key)
                self.assertEqual(fields(include)[key], '$(arg %s)' % parameter)

    def test_plugins_are_mutually_exclusive_and_installed(self):
        root = ET.parse(PACKAGE / 'urdf/scout_dynamics_plugins.xacro').getroot()
        ns = '{http://ros.org/wiki/xacro}'
        self.assertEqual(root.find(ns + 'unless').get('value'), '$(arg enable_ros_control)')
        self.assertEqual(root.find(ns + 'if').get('value'), '$(arg enable_ros_control)')
        cmake = (PACKAGE / 'CMakeLists.txt').read_text()
        self.assertIn('TARGETS scout_unicycle_plugin scout_wheel_diagnostics', cmake)
        self.assertIn('CATKIN_PACKAGE_SHARE_DESTINATION}/plugins', cmake)
        for name in ('scout_unicycle_plugin', 'scout_wheel_diagnostics'):
            self.assertIn('add_library(%s SHARED' % name, cmake)
        self.assertIn('xgc2_math 0.5.8 REQUIRED', cmake)

    def test_diagnostics_cannot_actuate(self):
        text = (PACKAGE / 'src/scout_wheel_diagnostics.cpp').read_text()
        for token in ('SetForce(', 'SetVelocity(', 'SetLinearVel(', 'SetAngularVel(', 'SetWorldPose('):
            self.assertNotIn(token, text)
        self.assertIn('collisions_[i]->WorldPose()', text)
        self.assertIn('wheelContactKinematics(', text)

    def test_unicycle_uses_consistent_link_velocity_and_no_pose_teleport(self):
        text = (PACKAGE / 'src/scout_unicycle_plugin.cpp').read_text()
        self.assertIn('origin_velocity + omega.Cross(offset)', text)
        self.assertNotIn('SetWorldPose(', text)
        self.assertIn('gate_->withCommand', text)
        self.assertIn('drive_->reset(plugin.now_)', text)
        self.assertIn('unicycle_virtual_wheels', text)
        self.assertNotIn('command_gain', text)

    def test_no_unknown_spawn_arguments(self):
        accepted = set(fields(launch('spawn_accurate.launch')))
        for name in ('accurate.launch', 'simple.launch', 'multi_accurate.launch'):
            for include in launch(name).findall("include[@file='$(dirname)/spawn_accurate.launch']"):
                self.assertFalse(set(fields(include)) - accepted, name)


if __name__ == '__main__':
    unittest.main()
