#!/usr/bin/env python3
"""Wiring checks complement the executable C++ state-machine tests."""
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class ControlContractTest(unittest.TestCase):
    def test_spawn_does_not_own_world_unpause(self):
        root = ET.parse(ROOT / 'launch/spawn_accurate.launch').getroot()
        spawns = [n for n in root.findall('node') if n.get('type') == 'spawn_model']
        self.assertEqual(len(spawns), 1)
        self.assertNotIn('-unpause', spawns[0].get('args', '').split())

    def test_input_and_periodic_output_use_hold_transaction(self):
        text = (ROOT / 'src/scout_skid_steer.cpp').read_text()
        receive = text.split('void ScoutSkidSteer::TwistCmdCallback(', 1)[1].split(
            'void ScoutSkidSteer::ControlTick(', 1)[0]
        tick = text.split('void ScoutSkidSteer::ControlTick(', 1)[1].split(
            'double ScoutSkidSteer::Clamp(', 1)[0]
        self.assertIn('hold_gate_.withCommand', receive)
        self.assertIn('hold_gate_.withCommand', tick)
        self.assertIn('command_dynamics_.Push', receive)
        self.assertNotIn('motor_fr_pub_.publish', receive)
        self.assertIn('command_dynamics_.Advance(ros::Time::now().toSec())', tick)
        self.assertIn('createWallTimer', text)
        self.assertIn('command_dynamics_.Reset()', text)

    def test_shutdown_drains_producers_before_unregistering(self):
        text = (ROOT / 'src/scout_skid_steer.cpp').read_text().split(
            'ScoutSkidSteer::~ScoutSkidSteer()', 1)[1].split(
            'void ScoutSkidSteer::SetupSubscription()', 1)[0]
        self.assertLess(text.index('control_timer_.stop()'), text.index('Hub::instance().remove'))
        self.assertLess(text.index('cmd_sub_.shutdown()'), text.index('Hub::instance().remove'))

    def test_udp_uses_draining_registry(self):
        text = (ROOT / 'include/xgc_chassis_hold/udp.hpp').read_text()
        self.assertIn('registry_.apply(robot, held)', text)
        self.assertIn('registry_.remove(gate)', text)
        self.assertNotIn('Gate *match(', text)


if __name__ == '__main__':
    unittest.main()
