#!/usr/bin/env python3
"""Lock the Scout launch defaults to the validated finite-response baseline."""

from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
EXPECTED = {
    "wheel_contact_mu1": "0.10",
    "wheel_contact_mu2": "1.0",
    "wheel_contact_fdir1": "0 0 1",
    "wheel_contact_slip1": "5.0",
    "wheel_contact_slip2": "0.0",
    "wheel_pid_p": "2.0",
    "wheel_pid_i": "0.0",
    "wheel_pid_d": "0.0",
    "command_gain": "1.10",
    "angular_command_gain": "1.15",
    "command_delay_s": "0.15",
    "command_time_constant_s": "0.15",
}


class ScoutStableDefaultsTest(unittest.TestCase):
    def test_simulation_owns_nonvisual_description_resources(self) -> None:
        for relative in (
            "launch/mini_description.launch",
            "rviz/navigation.rviz",
            "rviz/two_ugv_navigation.rviz",
            "urdf/empty.urdf",
            "urdf/mini.xacro",
            "urdf/scout_mini.gazebo",
            "urdf/scout_wheel.gazebo",
        ):
            self.assertTrue((PACKAGE / relative).is_file(), relative)

        forbidden = ("$(find scout_description)/launch", "$(find scout_description)/rviz", "$(find scout_description)/urdf")
        for directory in (PACKAGE / "launch", PACKAGE / "urdf"):
            for path in directory.iterdir():
                if path.is_file():
                    text = path.read_text()
                    for token in forbidden:
                        self.assertNotIn(token, text, path.name)

        model = (PACKAGE / "urdf" / "mini.xacro").read_text()
        self.assertIn("package://scout_description/meshes/", model)

    def test_accurate_launches_share_validated_defaults(self) -> None:
        for filename in ("accurate.launch", "multi_accurate.launch", "spawn_accurate.launch"):
            root = ET.parse(PACKAGE / "launch" / filename).getroot()
            defaults = {
                element.attrib["name"]: element.attrib.get("default")
                for element in root.findall("./arg")
            }
            for name, expected in EXPECTED.items():
                self.assertEqual(defaults.get(name), expected, f"{filename}: {name}")

    def test_controller_yaml_uses_stable_proportional_gain(self) -> None:
        text = (PACKAGE / "config" / "scout_mini_ros_control.yaml").read_text()
        self.assertEqual(text.count("p: 2.0"), 8)
        self.assertNotIn("p: 6.0", text)
        self.assertNotIn("p: 9.0", text)

    def test_sensor_simulation_is_opt_in_on_every_launch_path(self) -> None:
        for filename in (
            "accurate.launch",
            "multi_accurate.launch",
            "simple.launch",
            "spawn_accurate.launch",
            "spawn_simple.launch",
        ):
            root = ET.parse(PACKAGE / "launch" / filename).getroot()
            defaults = {
                element.attrib["name"]: element.attrib.get("default")
                for element in root.findall("./arg")
            }
            self.assertEqual(defaults.get("enable_lidar"), "false", filename)
            self.assertEqual(defaults.get("enable_rslidar"), "false", filename)
            self.assertEqual(defaults.get("enable_camera"), "false", filename)

        for filename in ("accurate.launch", "simple.launch", "spawn_accurate.launch", "spawn_simple.launch"):
            text = (PACKAGE / "launch" / filename).read_text()
            self.assertIn('name="enable_lidar" value="$(arg enable_lidar)"', text, filename)
            self.assertIn('name="enable_rslidar" value="$(arg enable_rslidar)"', text, filename)
            self.assertIn('name="enable_camera" value="$(arg enable_camera)"', text, filename)

    def test_imu_matches_agilex_field_effective_rate(self) -> None:
        # HI226 field-effective rate is 100 Hz. Do not match bag/hz ~200.
        root = ET.parse(PACKAGE / "urdf" / "scout_mini.gazebo").getroot()
        sensor = root.find(".//{*}sensor[@name='imu_sensor']")
        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.findtext("{*}update_rate"), "100.0")
        plugin = sensor.find("{*}plugin[@name='imu_plugin']")
        self.assertIsNotNone(plugin)
        self.assertEqual(plugin.findtext("{*}updateRateHZ"), "100.0")
        self.assertEqual(plugin.findtext("{*}topicName"), "imu/data_raw")

    def test_imu_matches_agilex_field_rest_noise(self) -> None:
        # rest_0 sample std @ 100 Hz, bag estimator-imu-check-20260819-220847.
        # Plugin gaussianNoise must stay 0; SDF noise is the contract.
        root = ET.parse(PACKAGE / "urdf" / "scout_mini.gazebo").getroot()
        sensor = root.find(".//{*}sensor[@name='imu_sensor']")
        self.assertIsNotNone(sensor)
        plugin = sensor.find("{*}plugin[@name='imu_plugin']")
        self.assertIsNotNone(plugin)
        self.assertEqual(plugin.findtext("{*}gaussianNoise"), "0.0")
        expected = {
            "angular_velocity": {"x": "0.0013", "y": "0.0011", "z": "0.0007"},
            "linear_acceleration": {"x": "0.025", "y": "0.020", "z": "0.062"},
        }
        imu = sensor.find("{*}imu")
        self.assertIsNotNone(imu)
        for quantity, axes in expected.items():
            block = imu.find("{*}%s" % quantity)
            self.assertIsNotNone(block, quantity)
            for axis, stddev in axes.items():
                noise = block.find("{*}%s/{*}noise" % axis)
                self.assertIsNotNone(noise, "%s.%s" % (quantity, axis))
                self.assertEqual(noise.get("type"), "gaussian", "%s.%s" % (quantity, axis))
                self.assertEqual(noise.findtext("{*}mean"), "0.0", "%s.%s mean" % (quantity, axis))
                self.assertEqual(noise.findtext("{*}stddev"), stddev, "%s.%s stddev" % (quantity, axis))
                self.assertIsNone(noise.find("{*}bias_mean"), "%s.%s extra bias" % (quantity, axis))

    def test_spawn_uses_explicit_stable_parameters_without_legacy_mapping(self) -> None:
        text = (PACKAGE / "launch" / "spawn_accurate.launch").read_text()
        self.assertNotIn("effective_wheel_", text)
        self.assertIn(
            'name="wheel_contact_mu1" value="$(arg wheel_contact_mu1)"',
            text,
        )
        self.assertIn(
            'name="wheel_contact_mu2" value="$(arg wheel_contact_mu2)"',
            text,
        )
        self.assertIn(
            'name="wheel_contact_fdir1" value="$(arg wheel_contact_fdir1)"',
            text,
        )
        self.assertIn(
            'name="wheel_contact_slip1" value="$(arg wheel_contact_slip1)"',
            text,
        )
        self.assertIn(
            'name="wheel_contact_slip2" value="$(arg wheel_contact_slip2)"',
            text,
        )
        self.assertEqual(text.count('value="$(arg wheel_pid_p)"'), 8)
        self.assertIn(
            'name="command_delay_s" type="double" value="$(arg command_delay_s)"',
            text,
        )
        self.assertIn(
            'name="command_time_constant_s" type="double" value="$(arg command_time_constant_s)"',
            text,
        )

    def test_wheel_friction_direction_is_fixed_to_the_axle(self) -> None:
        # Wheel-local z is the joint axis. Unlike local x, it does not rotate
        # through the contact plane as the wheel spins.
        model = (PACKAGE / "urdf" / "mini.xacro").read_text()
        self.assertIn('name="wheel_contact_fdir1" default="0 0 1"', model)
        self.assertIn('name="wheel_contact_mu1" default="0.10"', model)
        self.assertIn('name="wheel_contact_mu2" default="1.0"', model)

    def test_bridge_voltage_topic_matches_wheeltec_powervoltage(self) -> None:
        text = (PACKAGE / "src" / "sim_scout_status.cpp").read_text()
        self.assertIn('DeriveBridgeTopic(status_topic_, "PowerVoltage")', text)
        self.assertNotIn("scout/battery_voltage", text)


if __name__ == "__main__":
    unittest.main()
