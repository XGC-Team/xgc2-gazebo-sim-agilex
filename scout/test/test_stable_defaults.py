#!/usr/bin/env python3
"""Lock the Scout launch defaults to the validated finite-response baseline."""

from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
EXPECTED = {
    "wheel_contact_mu2": "0.10",
    "wheel_contact_slip2": "5.0",
    "wheel_pid_p": "6.0",
    "wheel_pid_i": "0.0",
    "wheel_pid_d": "0.0",
    "command_gain": "1.03",
    "angular_command_gain": "1.20",
}


class ScoutStableDefaultsTest(unittest.TestCase):
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
        self.assertEqual(text.count("p: 6.0"), 8)
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
            self.assertEqual(defaults.get("enable_camera"), "false", filename)

        for filename in ("accurate.launch", "simple.launch", "spawn_accurate.launch", "spawn_simple.launch"):
            text = (PACKAGE / "launch" / filename).read_text()
            self.assertIn('name="enable_lidar" value="$(arg enable_lidar)"', text, filename)
            self.assertIn('name="enable_camera" value="$(arg enable_camera)"', text, filename)

    def test_spawn_maps_only_the_known_unstable_legacy_triple(self) -> None:
        text = (PACKAGE / "launch" / "spawn_accurate.launch").read_text()
        legacy_guard = (
            "float(arg('wheel_contact_mu2')) == 0.35 and "
            "float(arg('wheel_contact_slip2')) == 0.1 and "
            "float(arg('wheel_pid_p')) == 9.0"
        )
        self.assertEqual(text.count(legacy_guard), 3)
        self.assertIn(
            'name="wheel_contact_mu2" value="$(arg effective_wheel_contact_mu2)"',
            text,
        )
        self.assertIn(
            'name="wheel_contact_slip2" value="$(arg effective_wheel_contact_slip2)"',
            text,
        )
        self.assertEqual(text.count('value="$(arg effective_wheel_pid_p)"'), 8)


if __name__ == "__main__":
    unittest.main()
