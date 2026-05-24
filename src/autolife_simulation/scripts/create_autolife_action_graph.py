import omni.graph.core as og
import omni.usd
from joint_state_bridge import setup_joint_state_bridge

graph_path = "/ActionGraph"
robot_path = "/World/autolife/root_joint"

stage = omni.usd.get_context().get_stage()
if stage.GetPrimAtPath(graph_path).IsValid():
    stage.RemovePrim(graph_path)


setup_joint_state_bridge(graph_path=graph_path, robot_path=robot_path)
