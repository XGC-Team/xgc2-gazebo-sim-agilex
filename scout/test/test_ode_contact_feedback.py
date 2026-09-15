#!/usr/bin/env python3
"""Source/portable adapter regressions; these do not certify Gazebo dynamics."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

P = Path(__file__).resolve().parents[1]
SOURCE = (P / 'src/scout_implicit_wheel_plugin.cpp').read_text()


class ContactFeedback(unittest.TestCase):
    def test_production_projection_numerically(self):
        compiler = shutil.which('g++')
        self.assertIsNotNone(compiler, 'g++ is required for the contact adapter regression')
        with tempfile.TemporaryDirectory(prefix='ode-contact-') as directory:
            binary = str(Path(directory) / 'contact-test')
            subprocess.run([compiler, '-std=c++11', '-Wall', '-Wextra', '-Werror',
                            '-pedantic', '-I' + str(P / 'include'),
                            str(P / 'test/ode_contact_feedback_test.cpp'), '-o', binary],
                           check=True, timeout=60)
            subprocess.run([binary], check=True, timeout=30)

    def test_pose_is_captured_before_physics_not_at_collection(self):
        self.assertIn('ConnectBeforePhysicsUpdate(', SOURCE)
        capture = SOURCE.split('void CaptureFeedbackFrame(', 1)[1].split('void Collect()', 1)[0]
        self.assertIn('wheels_[i]->WorldPose().Rot()', capture)
        collect = SOURCE.split('void Collect()', 1)[1].split('void PublishContacts(', 1)[0]
        self.assertNotIn('WorldPose()', collect)
        self.assertIn('contact->time.Double()', collect)
        self.assertIn('ode_contact::restore(frames[i],sample_time,now,first,', collect)
        self.assertIn('feedback_frames_={}', collect)
        self.assertIn('patches_[i].time=sample_time', collect)

    def test_filter_is_local_owned_and_registered_by_pointer(self):
        self.assertIn('std::map<std::string,gazebo::physics::CollisionPtr> collisions', SOURCE)
        self.assertIn('collisions.emplace(collisions_[i]->GetScopedName(),collisions_[i])', SOURCE)
        self.assertIn('if(topic.empty())throw', SOURCE)
        self.assertIn('if(!contact_sub_)throw', SOURCE)
        self.assertIn('RemoveFilter(filter_)', SOURCE)
        self.assertNotIn('SetNeverDropContacts', SOURCE)
        self.assertNotIn('std::max(0.0,force.Z())', SOURCE)

    def test_pause_reset_and_diagnostics(self):
        self.assertIn('ConnectPause(', SOURCE)
        self.assertIn('ConnectTimeReset(', SOURCE)
        self.assertIn('void InvalidateFeedback() { patches_={};feedback_frames_={}; }', SOURCE)
        self.assertGreaterEqual(SOURCE.count('InvalidateFeedback();'), 3)
        self.assertIn('simulation/implicit_contacts', SOURCE)
        for name in ('discard_no_frame', 'discard_time', 'discard_nonfinite',
                     'discard_nonhorizontal', 'discard_moving', 'discard_nonpositive'):
            self.assertIn(name, SOURCE)
        # Existing 14-column wheel diagnostics remain compatible with harness.
        self.assertIn('report.layout.dim[1].size=14', SOURCE)
        diagnostic = SOURCE.split('void PublishContacts(', 1)[1].split('void Update(', 1)[0]
        labels = re.search(r'dim\[1\].label="([^"]+)"', diagnostic).group(1).split(',')
        self.assertEqual(len(labels), 23)
        self.assertEqual(len(set(labels)), 23)


if __name__ == '__main__':
    unittest.main()
