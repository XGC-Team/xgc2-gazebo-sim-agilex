"""ROS-free xacro expansion tests; not a Gazebo or hardware acceptance."""
from pathlib import Path
import math,unittest,xml.etree.ElementTree as ET
import xacro
ROOT=Path(__file__).resolve().parents[1]
class PhysicalWheelContract(unittest.TestCase):
    def expand(self,**kwargs):
        mapping={'urdf_extras':str(ROOT/'urdf/empty.urdf')}
        mapping.update({k:str(v) for k,v in kwargs.items()})
        return ET.fromstring(xacro.process_file(str(ROOT/'urdf/mini.xacro'),mappings=mapping).toxml())
    def assert_model(self,doc,r,m,width,damping,offset):
        wheels=[e for e in doc.findall('link') if e.attrib['name'].endswith('_wheel_link')]
        self.assertEqual(len(wheels),4)
        for link in wheels:
            self.assertAlmostEqual(float(link.find('collision/geometry/cylinder').get('radius')),r,places=12)
            self.assertAlmostEqual(float(link.find('collision/geometry/cylinder').get('length')),width,places=12)
            inertial=link.find('inertial');self.assertAlmostEqual(float(inertial.find('mass').get('value')),m)
            self.assertAlmostEqual(float(inertial.find('origin').get('xyz').split()[2]),offset)
            I=inertial.find('inertia');self.assertAlmostEqual(float(I.get('izz')),m*r*r/2,places=12)
            self.assertAlmostEqual(float(I.get('ixx')),m*(3*r*r+width*width)/12,places=12)
            self.assertAlmostEqual(float(I.get('iyy')),m*(3*r*r+width*width)/12,places=12)
        for joint in doc.findall('joint'):
            if joint.get('type')=='continuous':
                self.assertAlmostEqual(float(joint.find('dynamics').get('damping')),damping)
    def test_legacy_physical_baseline_preserved(self):
        self.assert_model(self.expand(),.08,3.,.084,.1,0.)
    def test_parameter_change_reaches_collision_inertia_and_damping(self):
        self.assert_model(self.expand(wheel_radius=.1,wheel_mass=2.,wheel_width=.09,wheel_joint_damping=.03,wheel_inertial_axial_offset=.02),.1,2.,.09,.03,.02)
    def test_radius_is_forwarded_to_both_description_and_allocator(self):
        spawn=ET.parse(ROOT/'launch/spawn_accurate.launch').getroot()
        desc=next(e for e in spawn.findall('include') if e.get('file').endswith('mini_description.launch'))
        forwarded={e.get('name'):e.get('value') for e in desc.findall('arg')}
        for key in ('wheel_radius','wheel_mass','wheel_width','wheel_joint_damping','wheel_inertial_axial_offset'):
            self.assertEqual(forwarded[key],'$(arg '+key+')')
        text=(ROOT/'launch/mini_description.launch').read_text()
        self.assertIn('wheel_radius:=$(arg wheel_radius)',text)
    def test_delay_and_input_inertia_contract(self):
        for name in ('accurate.launch','spawn_accurate.launch'):
            root=ET.parse(ROOT/'launch'/name).getroot();args={e.get('name'):e.get('default') for e in root.findall('arg')}
            self.assertEqual(float(args['command_delay_s']),.005)
            self.assertFalse(any('time_constant' in k for k in args))
if __name__=='__main__':unittest.main(verbosity=2)
