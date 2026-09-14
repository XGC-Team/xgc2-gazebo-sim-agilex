"""Expanded physical wheel contract; no roscore/Gazebo/hardware is invoked."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
try:
    import xacro
except ImportError:
    xacro = None
ROOT = Path(__file__).resolve().parents[1]


class PhysicalWheelContract(unittest.TestCase):
    def expand(self, **kwargs):
        if xacro is None:
            self.skipTest('xacro absent; dedicated physical-contract CI requires it')
        mapping = {'urdf_extras': str(ROOT / 'urdf/empty.urdf')}
        mapping.update({k: str(v) for k, v in kwargs.items()})
        return ET.fromstring(xacro.process_file(str(ROOT / 'urdf/mini.xacro'), mappings=mapping).toxml())

    def assert_model(self, doc, radius, mass, width, damping, offset):
        wheels = [e for e in doc.findall('link') if e.attrib['name'].endswith('_wheel_link')]
        self.assertEqual(len(wheels), 4)
        for link in wheels:
            cylinder = link.find('collision/geometry/cylinder')
            self.assertAlmostEqual(float(cylinder.get('radius')), radius, places=12)
            self.assertAlmostEqual(float(cylinder.get('length')), width, places=12)
            inertia = link.find('inertial')
            self.assertAlmostEqual(float(inertia.find('mass').get('value')), mass)
            self.assertAlmostEqual(float(inertia.find('origin').get('xyz').split()[2]), offset)
            matrix = inertia.find('inertia')
            self.assertAlmostEqual(float(matrix.get('izz')), mass * radius**2 / 2, places=12)
            transverse = mass * (3 * radius**2 + width**2) / 12
            self.assertAlmostEqual(float(matrix.get('ixx')), transverse, places=12)
            self.assertAlmostEqual(float(matrix.get('iyy')), transverse, places=12)
        for joint in doc.findall('joint'):
            if joint.get('type') == 'continuous':
                self.assertAlmostEqual(float(joint.find('dynamics').get('damping')), damping)

    def test_legacy_physical_baseline_preserved(self):
        self.assert_model(self.expand(), .08, 3., .084, .1, 0.)

    def test_nondefault_parameters_reach_the_expanded_model(self):
        self.assert_model(self.expand(wheel_radius=.1, wheel_mass=2., wheel_width=.09,
                                     wheel_joint_damping=.03, wheel_inertial_axial_offset=.02),
                          .1, 2., .09, .03, .02)

    def test_radius_is_forwarded_to_description_and_allocator(self):
        spawn = ET.parse(ROOT / 'launch/spawn_accurate.launch').getroot()
        description = next(e for e in spawn.findall('include') if e.get('file').endswith('mini_description.launch'))
        forwarded = {e.get('name'): e.get('value') for e in description.findall('arg')}
        for key in ('wheel_radius', 'wheel_mass', 'wheel_width', 'wheel_joint_damping', 'wheel_inertial_axial_offset'):
            self.assertEqual(forwarded[key], '$(arg ' + key + ')')
        self.assertIn('wheel_radius:=$(arg wheel_radius)', (ROOT / 'launch/mini_description.launch').read_text())

    def test_delay_and_input_inertia_contract(self):
        for name in ('accurate.launch', 'spawn_accurate.launch'):
            root = ET.parse(ROOT / 'launch' / name).getroot()
            args = {e.get('name'): e.get('default') for e in root.findall('arg')}
            self.assertEqual(float(args['command_delay_s']), .005)
            self.assertFalse(any('time_constant' in key for key in args))


if __name__ == '__main__':
    unittest.main(verbosity=2)
