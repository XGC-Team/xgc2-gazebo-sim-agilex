#!/usr/bin/env python3
"""Check the actual xacro -> SDFormat boundary, including parameter overrides."""
from pathlib import Path
import hashlib
import math
import os
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]


class ScoutSurfaceTest(unittest.TestCase):
    def test_collision_matches_retained_tire_geometry(self):
        # Compare actual expanded collision placement with the retained CAD,
        # rather than assuming a URDF joint frame is the tire midpoint.
        if os.environ.get('SCOUT_WHEEL_MESH'):
            mesh = Path(os.environ['SCOUT_WHEEL_MESH'])
        else:
            description = Path(subprocess.check_output(
                ['rospack', 'find', 'scout_description'], text=True).strip())
            mesh = description / 'meshes/wheel.dae'
        self.assertEqual(hashlib.sha256(mesh.read_bytes()).hexdigest(),
                         'c64ae34e44118f07d718d54199107ce430a006d1cd9fd96bb80968954b5a8ce7',
                         'Retained CAD changed; review tire geometry before updating this contract')
        ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
        tire = ET.parse(mesh).getroot().find(".//c:geometry[@id='geometry4']/c:mesh", ns)
        position_id = tire.find("c:vertices/c:input[@semantic='POSITION']", ns).get('source')[1:]
        coordinates = list(map(float, tire.find(
            "c:source[@id='%s']/c:float_array" % position_id, ns).text.split()))
        axial = coordinates[2::3]
        tire_center = (min(axial) + max(axial)) / 2
        tire_width = max(axial) - min(axial)
        robot = ET.fromstring(subprocess.check_output(
            ['xacro', str(PACKAGE / 'urdf/mini.xacro')], text=True))
        for wheel in robot.findall('link'):
            if not wheel.get('name', '').endswith('_wheel_link'):
                continue
            with self.subTest(wheel=wheel.get('name')):
                collision = wheel.find('collision')
                xyz = list(map(float, collision.find('origin').get('xyz').split()))
                self.assertAlmostEqual(xyz[2], tire_center, places=8)
                cylinder = collision.find('geometry/cylinder')
                self.assertLess(abs(float(cylinder.get('length')) - tire_width), .0005)
                self.assertAlmostEqual(float(cylinder.get('radius')), .08, places=8)
                # Keep the mounting and visual frames; do not move the visible
                # wheels inward to hide a misplaced collision.
                self.assertEqual(wheel.find('visual/origin').get('xyz'), '0 0 0')
                joint = next(j for j in robot.findall('joint')
                             if j.find('child').get('link') == wheel.get('name'))
                mount = list(map(float, joint.find('origin').get('xyz').split()))
                self.assertAlmostEqual(abs(mount[1]), .2082515, places=8)

    def test_every_wheel_retains_its_contact_parameters(self):
        self.assertIsNotNone(shutil.which('xacro'), 'ROS xacro is required')
        self.assertIsNotNone(shutil.which('gz'), 'Gazebo SDFormat converter is required')
        for overrides in ({}, {'wheel_contact_slip1': '.125', 'wheel_contact_slip2': '.375'}):
            with self.subTest(overrides=overrides), tempfile.TemporaryDirectory() as directory:
                urdf = subprocess.check_output([
                    'xacro', str(PACKAGE / 'urdf/mini.xacro'),
                    *[name + ':=' + value for name, value in overrides.items()],
                ], text=True)
                robot = ET.fromstring(urdf)
                joints = [joint for joint in robot.findall('joint')
                          if joint.get('name', '').endswith('_wheel')]
                self.assertEqual(len(joints), 4)
                for joint in joints:
                    # Positive rotation must be about body +Y, without camber.
                    roll, pitch, yaw = map(float, joint.find('origin').get('rpy').split())
                    axis = list(map(float, joint.find('axis').get('xyz').split()))
                    self.assertAlmostEqual(pitch, 0., places=12)
                    self.assertAlmostEqual(yaw, 0., places=12)
                    self.assertAlmostEqual(axis[0], 0., places=12)
                    self.assertAlmostEqual(axis[1], 0., places=12)
                    self.assertAlmostEqual(-math.sin(roll)*axis[2], 1., places=12)
                    self.assertAlmostEqual(math.cos(roll)*axis[2], 0., places=12)
                path = Path(directory) / 'scout.urdf'
                path.write_text(urdf)
                sdf = ET.fromstring(subprocess.check_output(['gz', 'sdf', '-p', str(path)], text=True))
                wheels = [link for link in sdf.findall('.//link') if link.get('name', '').endswith('_wheel_link')]
                self.assertEqual(len(wheels), 4)
                for wheel in wheels:
                    collisions = wheel.findall('collision')
                    self.assertEqual(len(collisions), 1, wheel.get('name'))
                    pose = list(map(float, collisions[0].findtext('pose').split()))
                    # gz sdf -p serializes poses with six decimal places.
                    self.assertAlmostEqual(pose[2], .039638344, delta=1e-6)
                    ode = collisions[0].find('surface/friction/ode')
                    self.assertIsNotNone(ode)
                    expected = {'mu': .16, 'mu2': 1., 'slip1': 0., 'slip2': 0.}
                    for name in ('slip1', 'slip2'):
                        expected[name] = float(overrides.get('wheel_contact_' + name, expected[name]))
                    for name, value in expected.items():
                        self.assertIsNotNone(ode.find(name), wheel.get('name') + ': missing ' + name)
                        self.assertAlmostEqual(float(ode.findtext(name)), value)
                    self.assertEqual(ode.findtext('fdir1').split(), ['0', '0', '1'])
                    self.assertIsNone(wheel.find('slip1'), 'slip must not leak into the link')


if __name__ == '__main__':
    unittest.main()
