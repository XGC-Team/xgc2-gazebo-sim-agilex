#!/usr/bin/env python3
"""Check local launch composition and retired spawn packaging without any peer."""
import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class ParallelLaunchTest(unittest.TestCase):
    def test_launches_have_no_peer_start_barriers(self):
        paths = list(ROOT.rglob("*.launch"))
        self.assertTrue(paths)
        for path in paths:
            for element in ET.parse(path).getroot().iter():
                for value in element.attrib.values():
                    self.assertFalse(re.search(r"\bsleep\b|wait_for_|waitForService|await-service", value), str(path))
                if element.tag == "node":
                    self.assertNotEqual(element.get("required"), "true", str(path))
                    self.assertNotEqual(element.get("respawn"), "true", str(path))
                    if element.get("type") == "spawn_robot_model":
                        self.assertNotRegex(element.get("args", ""), "controller|scene_service|scene_adapter")

    def test_retired_spawn_definition_is_not_packaged(self):
        retired = "process-definitions/scout-gazebo-robot.json"
        self.assertFalse((ROOT / retired).exists())
        for relative in (".xgc2/product.yml", ".xgc2/scripts/package_debs.sh",
                         ".xgc2/scripts/check_installed_packages.sh"):
            with self.subTest(path=relative):
                self.assertNotIn(retired, (ROOT / relative).read_text())


if __name__ == "__main__":
    unittest.main()
