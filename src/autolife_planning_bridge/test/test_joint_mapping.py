import unittest

import numpy as np

from autolife_planning_bridge.joint_mapping import (
    CONTROL_JOINTS,
    PLANNING_JOINTS,
    chain_factory_args,
    chain_joint_names,
    infer_group,
    merge_goal_positions,
    resolve_chain_key,
    state_to_planning_vector,
)


class TestJointMapping(unittest.TestCase):
    def test_base_joint_mapping_to_planning_vector(self):
        positions = {name: float(index) for index, name in enumerate(CONTROL_JOINTS)}

        config, missing = state_to_planning_vector(positions)

        self.assertEqual(missing, [])
        self.assertIsNotNone(config)
        self.assertEqual(config.shape, (len(PLANNING_JOINTS),))
        self.assertEqual(
            config[PLANNING_JOINTS.index("Joint_Virtual_X")],
            positions["Joint_Ground_Vehicle_X"],
        )
        self.assertEqual(
            config[PLANNING_JOINTS.index("Joint_Virtual_Theta")],
            positions["Joint_Ground_Vehicle_Z"],
        )

    def test_group_inference(self):
        self.assertEqual(infer_group(["Joint_Left_Elbow"]), "autolife_left_arm")
        self.assertEqual(infer_group(["Joint_Right_Elbow"]), "autolife_right_arm")
        self.assertEqual(
            infer_group(["Joint_Left_Elbow", "Joint_Right_Elbow"]),
            "autolife_dual_arm",
        )
        self.assertEqual(infer_group(["Joint_Neck_Yaw"]), "autolife_body")
        self.assertEqual(infer_group(["Joint_Waist_Yaw"]), "autolife_body")

    def test_chain_resolution(self):
        self.assertEqual(resolve_chain_key("whole_body", "left"), "whole_body_left")
        self.assertEqual(
            chain_factory_args("whole_body_base_right"),
            ("whole_body_base", "right"),
        )
        self.assertIn("Joint_Left_Elbow", chain_joint_names("left_arm"))

    def test_merge_goal_positions(self):
        start = np.zeros(len(PLANNING_JOINTS))

        target = merge_goal_positions(start, ["Joint_Left_Elbow"], [1.2])

        self.assertEqual(target[PLANNING_JOINTS.index("Joint_Left_Elbow")], 1.2)
        self.assertEqual(np.count_nonzero(target), 1)


if __name__ == "__main__":
    unittest.main()
