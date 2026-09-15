#!/usr/bin/env python3
"""Checks the new force backend's source wiring, not Gazebo dynamic validity."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
P=Path(__file__).resolve().parents[1]
S=(P/'src/scout_implicit_wheel_plugin.cpp').read_text()
def fields(e,attr='value'):return {a.get('name'):a.get(attr) for a in e.findall('arg')}
class Wiring(unittest.TestCase):
    def test_no_kinematic_overwrite(self):
        for token in ('SetLinearVel(', 'SetAngularVel(', 'SetWorldPose(', 'SetVelocity('):self.assertNotIn(token,S)
        self.assertIn('AddForceAtWorldPosition(',S);self.assertIn('SetForce(',S)
        self.assertIn('implicitWheelContactStep(input,memory_,parameters_)',S)
    def test_no_double_reaction_or_legacy_actuator(self):
        root=ET.parse(P/'launch/spawn_implicit_brush.launch').getroot()
        self.assertFalse(any(e.get('type') in ('spawner','scout_skid_steer_controller') for e in root.findall('node')))
        self.assertEqual(fields(root.find('include'))['enable_ros_control'],'false')
        for name in ('SetMuPrimary(0)','SetMuSecondary(0)','SetMuTorsion(0)','SetDamping(0,0)'):self.assertIn(name,S)
        self.assertNotIn('base_->AddForce',S)
    def test_command_delay_but_no_added_lag(self):
        self.assertIn('DelayedPlanarVelocity({delay,0,0})',S)
        self.assertIn('command_delay_s',S)
        self.assertIn('nominal_radius_[i]=cylinder->GetRadius()',S)
        self.assertNotIn('/.08',S)
    def test_normal_feedback_is_local_and_fresh(self):
        for token in ('CreateFilter(', 'GetContactCount()', 'body1Force', 'body2Force','1.5*h','patches_={}'):self.assertIn(token,S)
        self.assertNotIn('SetNeverDropContacts',S)
        self.assertNotIn('9.81 / 4',S)
    def test_force_geometry_and_whole_assembly_inertia(self):
        for token in ('model_->GetLinks()', 'WorldInertiaMatrix()', 'GlobalAxis(0)', 'Anchor(0)', 'axis.Cross(point[i]-anchor)'):self.assertIn(token,S)
    def test_force_failure_and_hold_are_serialized(self):
        self.assertIn('gate_->withCommand',S)
        self.assertIn('Fault(e.what())',S)
        callback=S.split('static void HoldThunk',1)[1].split('void Command',1)[0]
        self.assertNotIn('SetForce(',callback)
        self.assertIn('delayed_->reset',callback)
    def test_dispatch_rejects_unknown_and_impossible_modes(self):
        root=ET.parse(P/'launch/spawn_selectable.launch').getroot()
        expr=fields(root)['implicit_brush_selected'][7:-1]
        def selected(mode,backend):return eval(expr,{'__builtins__':{}},{'arg':lambda key:{'dynamics_mode':mode,'wheel_physics_backend':backend}[key]})
        self.assertTrue(selected('wheel_physics','implicit_brush'))
        self.assertFalse(selected('wheel_physics','ode'));self.assertFalse(selected('unicycle','ode'))
        for a,b in [('wheel_physics','typo'),('typo','ode'),('unicycle','implicit_brush')]:
            with self.assertRaises(KeyError):selected(a,b)
        branches=root.findall('include')
        self.assertEqual(len(branches),2)
        self.assertEqual(branches[0].get('unless'),branches[1].get('if'))
    def test_forwarding_and_unity_candidate_calibration(self):
        root=ET.parse(P/'launch/spawn_selectable.launch').getroot()
        defaults=fields(root,'default')
        self.assertEqual(defaults['brush_allocation_gain'],'1.0')
        self.assertEqual(defaults['brush_yaw_calibration'],'1.0')
        child=ET.parse(P/'launch/spawn_implicit_brush.launch').getroot()
        passed=fields(root.findall('include')[1]);self.assertFalse(set(passed)-set(fields(child)))
        self.assertEqual(fields(root.findall('include')[0])['command_gain'],'$(arg command_gain)')
    def test_plugin_path_and_install(self):
        root=ET.parse(P/'urdf/scout_implicit_wheel.xacro').getroot()
        self.assertEqual(root.find('gazebo/plugin').get('filename'),'libscout_implicit_wheel_plugin.so')
        cmake=(P/'cmake/implicit_wheel.cmake').read_text()
        self.assertIn('SHARED',cmake);self.assertIn('install(TARGETS scout_implicit_wheel_plugin',cmake)
if __name__=='__main__':unittest.main()
