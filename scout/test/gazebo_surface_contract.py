#!/usr/bin/env python3
"""Check the actual xacro -> SDFormat boundary, including parameter overrides."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]


class ScoutSurfaceTest(unittest.TestCase):
    def test_every_wheel_retains_its_contact_parameters(self):
        self.assertIsNotNone(shutil.which('xacro'), 'ROS xacro is required')
        self.assertIsNotNone(shutil.which('gz'), 'Gazebo SDFormat converter is required')
        for overrides in ({}, {'wheel_contact_slip1': '.125', 'wheel_contact_slip2': '.375'}):
            with self.subTest(overrides=overrides), tempfile.TemporaryDirectory() as directory:
                urdf = subprocess.check_output([
                    'xacro', str(PACKAGE / 'urdf/mini.xacro'),
                    *[name + ':=' + value for name, value in overrides.items()],
                ], text=True)
                path = Path(directory) / 'scout.urdf'
                path.write_text(urdf)
                sdf = ET.fromstring(subprocess.check_output(['gz', 'sdf', '-p', str(path)], text=True))
                wheels = [link for link in sdf.findall('.//link') if link.get('name', '').endswith('_wheel_link')]
                self.assertEqual(len(wheels), 4)
                for wheel in wheels:
                    collisions = wheel.findall('collision')
                    self.assertEqual(len(collisions), 1, wheel.get('name'))
                    ode = collisions[0].find('surface/friction/ode')
                    self.assertIsNotNone(ode)
                    expected = {'mu': .1, 'mu2': 1., 'slip1': 0., 'slip2': 0.}
                    for name in ('slip1', 'slip2'):
                        expected[name] = float(overrides.get('wheel_contact_' + name, expected[name]))
                    for name, value in expected.items():
                        self.assertIsNotNone(ode.find(name), wheel.get('name') + ': missing ' + name)
                        self.assertAlmostEqual(float(ode.findtext(name)), value)
                    self.assertEqual(ode.findtext('fdir1').split(), ['0', '0', '1'])
                    self.assertIsNone(wheel.find('slip1'), 'slip must not leak into the link')


if __name__ == '__main__':
    unittest.main()
