"""Pure geometry regression: no ROS nodes, SDK connection or motion."""

import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro

ROOT = Path(__file__).resolve().parents[2]


class GeometryTest(unittest.TestCase):
    def test_default_mounts_reproduce_source_robot_urdf(self):
        tree = ET.fromstring(
            xacro.process_file(
                str(ROOT / "iimt_dual_description/urdf/iimt_dual.urdf.xacro")
            ).toxml()
        )
        left = tree.find("joint[@name='world_to_left_base']/origin")
        right = tree.find("joint[@name='left_to_right_base']/origin")
        xyz = [float(v) for v in left.get("xyz").split()]
        rpy = [float(v) for v in left.get("rpy").split()]
        offset = [float(v) for v in right.get("xyz").split()]
        self.assertEqual(xyz, [-0.0008297, -0.538, 0.0818739])
        self.assertEqual(rpy, [0.0, 0.0, -0.6981317008])
        self.assertEqual([float(v) for v in right.get("rpy").split()], [0.0] * 3)
        c, s = math.cos(rpy[2]), math.sin(rpy[2])
        world_right = [
            xyz[0] + c * offset[0] - s * offset[1],
            xyz[1] + s * offset[0] + c * offset[1],
            xyz[2] + offset[2],
        ]
        # Fixed joint origins copied from workspace robot.urdf; test stays portable.
        for actual, expected in zip(world_right, [-0.0008297, -0.00059, -0.0001261]):
            self.assertAlmostEqual(actual, expected, places=11)

    def test_vendor_geometry_and_relative_mounts(self):
        tree = ET.fromstring(
            xacro.process_file(
                str(ROOT / "iimt_dual_description/urdf/iimt_dual.urdf.xacro"),
                mappings={
                    "left_base_xyz": "0.1 0.2 0.3",
                    "left_base_rpy": "0 0 1.5707963267948966",
                    "right_base_xyz": "1.2 0 0",
                    "right_base_rpy": "0 0 0.5",
                },
            ).toxml()
        )
        joints = tree.findall("joint")
        self.assertEqual(len([j for j in joints if j.get("type") == "revolute"]), 12)
        self.assertEqual(len(tree.findall("link")), 15)
        self.assertFalse(tree.findall("ros2_control"))
        mounts = {j.get("name"): j for j in joints if j.get("type") == "fixed"}
        self.assertEqual(
            mounts["world_to_left_base"].find("parent").get("link"), "world"
        )
        right = mounts["left_to_right_base"]
        self.assertEqual(right.find("parent").get("link"), "left_base_link")
        self.assertEqual(right.find("origin").get("xyz"), "1.2 0 0")
        # At yaw pi/2 a relative +x offset is world +y, not world +x.
        yaw = float(mounts["world_to_left_base"].find("origin").get("rpy").split()[2])
        self.assertAlmostEqual(0.1 + 1.2 * math.cos(yaw), 0.1)
        self.assertAlmostEqual(0.2 + 1.2 * math.sin(yaw), 1.4)
        vendor = ET.parse(ROOT / "iimt_ros2_description/urdf/iimt5.urdf").getroot()
        for side in ("left", "right"):
            for original in vendor:
                actual = tree.find(
                    f"{original.tag}[@name='{side}_{original.get('name')}']"
                )
                self.assertIsNotNone(actual)
                # Remove prefixes and compare all geometry/inertia/limits numerically unchanged.
                for element in actual.iter():
                    for attr in ("name", "link"):
                        value = element.get(attr)
                        if value and value.startswith(side + "_"):
                            element.set(attr, value[len(side) + 1 :])

                def signature(node):
                    return (node.tag, node.attrib, [signature(child) for child in node])

                self.assertEqual(signature(actual), signature(original))

    def test_collision_exclusions_only_within_each_arm(self):
        srdf = ET.parse(ROOT / "iimt_dual_moveit_config/srdf/iimt_dual.srdf").getroot()
        pairs = srdf.findall("disable_collisions")
        self.assertEqual(len(pairs), 16)
        for pair in pairs:
            self.assertEqual(
                pair.get("link1").split("_")[0], pair.get("link2").split("_")[0]
            )
        both = srdf.find("group[@name='both_arms']")
        self.assertEqual({g.get("name") for g in both}, {"left_arm", "right_arm"})
        for side in ("left", "right"):
            chain = srdf.find(f"group[@name='{side}_arm']/chain")
            self.assertEqual(chain.get("base_link"), side + "_base_link")
            self.assertEqual(chain.get("tip_link"), side + "_wrist_3_link")


if __name__ == "__main__":
    unittest.main()
