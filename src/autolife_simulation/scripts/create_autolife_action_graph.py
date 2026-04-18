import omni.graph.core as og
import omni.usd


graph_path = "/ActionGraph"

stage = omni.usd.get_context().get_stage()
if stage.GetPrimAtPath(graph_path).IsValid():
    stage.RemovePrim(graph_path)

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
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),

            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),

            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
            ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
        ],
        og.Controller.Keys.SET_VALUES: [
            # Providing path to /autolife robot to Articulation Controller node
            # Providing the robot path is equivalent to setting the targetPrim in Articulation Controller node
            # ("ArticulationController.inputs:usePath", True),      # if you are using an older version of Isaac Sim, you can  uncomment this line
            ("ArticulationController.inputs:robotPath", "/World/autolife/root_joint"),
            ("PublishJointState.inputs:targetPrim", "/World/autolife/root_joint")
        ],
    },
)