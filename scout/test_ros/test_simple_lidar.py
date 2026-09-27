#!/usr/bin/env python3
"""Render the optional Scout lidar using the shared xacro macro."""

from __future__ import annotations

import os
import shutil
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
POSE_DEFAULT = "0 0 0.35 0 0 0"
SENSOR_ARGS = {
    "enable_simple_lidar": "false",
    "simple_lidar_pose": POSE_DEFAULT,
}


def installed_simple_lidar_package() -> Path:
    package = Path(subprocess.check_output(
        ["rospack", "find", "xgc2_simple_lidar"], text=True
    ).strip()).resolve()
    roscpp = Path(subprocess.check_output(
        ["rospack", "find", "roscpp"], text=True
    ).strip()).resolve()
    if package.parent != roscpp.parent:
        raise RuntimeError("xgc2_simple_lidar must resolve from the installed ROS share directory")
    if not (package / "models" / "sensor.xacro").is_file():
        raise RuntimeError(f"installed shared xacro macro missing: {package}")
    return package


def launch(path: str) -> ET.Element:
    return ET.parse(PACKAGE / "launch" / path).getroot()


def arg_defaults(root: ET.Element) -> dict[str, str | None]:
    return {
        item.attrib["name"]: item.attrib.get("default")
        for item in root.findall("./arg")
    }


def include_args(include: ET.Element) -> dict[str, str | None]:
    return {
        item.attrib["name"]: item.attrib.get("value")
        for item in include.findall("./arg")
    }


def without_simple_lidar(root: ET.Element) -> bytes:
    clone = ET.fromstring(ET.tostring(root))
    for gazebo in list(clone.findall("./gazebo")):
        for sensor in list(gazebo.findall("./sensor")):
            if sensor.attrib.get("name") == "simple_lidar":
                gazebo.remove(sensor)
        if not list(gazebo):
            clone.remove(gazebo)
    for element in clone.iter():
        if element.text is not None and not element.text.strip():
            element.text = ""
        if element.tail is not None and not element.tail.strip():
            element.tail = ""
    return ET.tostring(clone, encoding="utf-8")


class ScoutSimpleLidarTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.xacro = shutil.which("xacro")
        if not cls.xacro:
            raise RuntimeError("xacro executable is required for the rendered model test")
        cls.shared_package = installed_simple_lidar_package()

    def render(self, args: list[str], *, shared_package_available: bool) -> ET.Element:
        # The Scout package path is needed for its existing ros_control macro.
        # The shared package path is deliberately omitted in the disabled case.
        paths = [str(PACKAGE.parent)]
        if shared_package_available:
            paths.append(str(self.shared_package.parent))
        env = os.environ.copy()
        env["ROS_PACKAGE_PATH"] = os.pathsep.join(paths)
        env["CMAKE_PREFIX_PATH"] = ""
        command = [self.xacro, str(PACKAGE / "urdf" / "mini.xacro"), *args]
        result = subprocess.run(command, env=env, text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        return ET.fromstring(result.stdout)

    def test_default_renders_without_the_shared_package(self) -> None:
        root = self.render(
            [
                "ns:=ugv1",
                "enable_ros_control:=true",
                f"urdf_extras:={PACKAGE / 'urdf' / 'scout_dynamics_plugins.xacro'}",
            ],
            shared_package_available=False,
        )
        self.assertEqual(root.findall(".//sensor[@name='simple_lidar']"), [])
        self.assertEqual(root.findall(".//plugin[@filename='libxgc2_simple_lidar.so']"), [])

    def test_enabled_sensor_uses_public_macro_pose_and_robot_namespace(self) -> None:
        root = self.render(
            ["enable_simple_lidar:=true", "ns:=ugv1"],
            shared_package_available=True,
        )
        sensors = root.findall("./gazebo[@reference='base_link']/sensor[@name='simple_lidar']")
        self.assertEqual(len(sensors), 1)
        sensor = sensors[0]
        self.assertEqual(sensor.attrib.get("type"), "gpu_ray")
        self.assertEqual(sensor.findtext("pose"), POSE_DEFAULT)
        plugin = sensor.find("plugin")
        self.assertIsNotNone(plugin)
        self.assertEqual(plugin.attrib.get("filename"), "libxgc2_simple_lidar.so")
        self.assertEqual(plugin.findtext("robotNamespace"), "ugv1")

    def test_two_robot_namespaces_resolve_to_separate_topics(self) -> None:
        resolved_topics = []
        for namespace in ("ugv1", "ugv2"):
            root = self.render(
                ["enable_simple_lidar:=true", f"ns:={namespace}"],
                shared_package_available=True,
            )
            sensors = root.findall("./gazebo[@reference='base_link']/sensor[@name='simple_lidar']")
            self.assertEqual(len(sensors), 1)
            plugin = sensors[0].find("plugin")
            self.assertIsNotNone(plugin)
            self.assertEqual(plugin.findtext("robotNamespace"), namespace)
            resolved_topics.append(f"/{namespace}/simple_lidar/points")
        self.assertEqual(len(set(resolved_topics)), 2)

    def test_custom_pose_is_forwarded_to_the_shared_sensor(self) -> None:
        pose = "0.12 -0.03 0.41 0.0 0.0 1.57"
        root = self.render(
            ["enable_simple_lidar:=true", "ns:=ugv1", f"simple_lidar_pose:={pose}"],
            shared_package_available=True,
        )
        sensor = root.find("./gazebo[@reference='base_link']/sensor[@name='simple_lidar']")
        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.findtext("pose"), pose)

    def test_existing_lidar_switches_remain_separate(self) -> None:
        root = self.render(
            ["enable_lidar:=true", "enable_rslidar:=true", "ns:=ugv1"],
            shared_package_available=False,
        )
        self.assertEqual(len(root.findall(".//sensor[@name='laser_sensor']")), 1)
        self.assertEqual(len(root.findall(".//sensor[@name='helios16']")), 1)
        self.assertEqual(root.findall(".//sensor[@name='simple_lidar']"), [])

        manifest = ET.parse(PACKAGE / "package.xml").getroot()
        self.assertEqual(
            [item.text for item in manifest.findall("./test_depend") if item.text == "xgc2_simple_lidar"],
            ["xgc2_simple_lidar"],
        )

    def test_simple_lidar_does_not_change_other_rendered_model_parameters(self) -> None:
        dynamics_args = [
            "ns:=ugv1",
            "enable_ros_control:=true",
            f"urdf_extras:={PACKAGE / 'urdf' / 'scout_dynamics_plugins.xacro'}",
            "wheelbase:=0.48",
            "wheel_track:=0.42",
            "wheel_radius:=0.081",
            "wheel_mass:=3.2",
            "wheel_width:=0.086",
            "wheel_joint_damping:=0.13",
            "wheel_inertial_axial_offset:=0.002",
            "wheel_effort_limit:=5.5",
            "wheel_velocity_limit:=24.0",
            "wheel_contact_mu1:=0.22",
            "wheel_contact_mu2:=0.95",
            "wheel_contact_fdir1:=0 1 0",
            "wheel_contact_slip1:=0.01",
            "wheel_contact_slip2:=0.02",
        ]
        baseline = self.render(dynamics_args, shared_package_available=False)
        enabled = self.render(
            [*dynamics_args, "enable_simple_lidar:=true"],
            shared_package_available=True,
        )
        self.assertEqual(without_simple_lidar(enabled), without_simple_lidar(baseline))

    def test_launch_args_reach_each_model_spawn_path(self) -> None:
        for filename in (
            "accurate.launch",
            "helios16.launch",
            "implicit_brush.launch",
            "mini_description.launch",
            "multi_accurate.launch",
            "simple.launch",
            "spawn_accurate.launch",
            "spawn_implicit_brush.launch",
            "spawn_selectable.launch",
        ):
            for name, expected in SENSOR_ARGS.items():
                self.assertEqual(arg_defaults(launch(filename)).get(name), expected, filename)

        description = launch("mini_description.launch")
        xacro_command = description.find("./param[@name='$(arg robot_description_param)']").attrib["command"]
        self.assertIn("enable_simple_lidar:=$(arg enable_simple_lidar)", xacro_command)
        self.assertIn("simple_lidar_pose:='$(arg simple_lidar_pose)'", xacro_command)

        routes = {
            "accurate.launch": ("$(dirname)/spawn_accurate.launch", "enable_simple_lidar", "simple_lidar_pose"),
            "simple.launch": ("$(dirname)/spawn_accurate.launch", "enable_simple_lidar", "simple_lidar_pose"),
            "helios16.launch": ("$(dirname)/accurate.launch", "enable_simple_lidar", "simple_lidar_pose"),
            "implicit_brush.launch": ("$(dirname)/spawn_selectable.launch", "enable_simple_lidar", "simple_lidar_pose"),
            "spawn_accurate.launch": ("$(dirname)/mini_description.launch", "enable_simple_lidar", "simple_lidar_pose"),
            "spawn_implicit_brush.launch": ("$(dirname)/mini_description.launch", "enable_simple_lidar", "simple_lidar_pose"),
            "spawn_selectable.launch": ("$(dirname)/spawn_accurate.launch", "enable_simple_lidar", "simple_lidar_pose"),
        }
        for filename, (target, *names) in routes.items():
            includes = [item for item in launch(filename).findall("./include") if item.attrib.get("file") == target]
            self.assertTrue(includes, filename)
            for include in includes:
                forwarded = include_args(include)
                for name in names:
                    self.assertEqual(forwarded.get(name), f"$(arg {name})", f"{filename}: {name}")

        selectable = launch("spawn_selectable.launch")
        for target in (
            "$(dirname)/spawn_accurate.launch",
            "$(dirname)/spawn_implicit_brush.launch",
        ):
            matching = [item for item in selectable.findall("./include") if item.attrib.get("file") == target]
            self.assertEqual(len(matching), 1, target)
            forwarded = include_args(matching[0])
            for name in SENSOR_ARGS:
                self.assertEqual(forwarded.get(name), f"$(arg {name})", f"{target}: {name}")

        multi = launch("multi_accurate.launch")
        defaults = arg_defaults(multi)
        includes = [item for item in multi.findall("./include") if item.attrib.get("file") == "$(dirname)/spawn_accurate.launch"]
        self.assertEqual(len(includes), 2)
        for index, include in enumerate(includes, start=1):
            prefix = f"ugv{index}_"
            for name, expected in SENSOR_ARGS.items():
                robot_name = prefix + name
                self.assertEqual(defaults.get(robot_name), f"$(arg {name})", robot_name)
                self.assertEqual(include_args(include).get(name), f"$(arg {robot_name})", robot_name)


if __name__ == "__main__":
    unittest.main()
