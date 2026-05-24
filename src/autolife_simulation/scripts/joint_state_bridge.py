import omni.graph.core as og


def setup_joint_state_bridge(
    graph_path="/ActionGraph",
    robot_path="/World/autolife/root_joint",
    joint_states_topic="/joint_states",
    joint_command_topic="/autolife/joint_command",
):
    import usdrt

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),

                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),

                ("SubscribeJointState.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot_path)]),
                ("PublishJointState.inputs:topicName", joint_states_topic),
                ("SubscribeJointState.inputs:topicName", joint_command_topic),
                ("ArticulationController.inputs:robotPath", robot_path),
            ],
        },
    )
