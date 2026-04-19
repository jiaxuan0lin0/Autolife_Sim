from pxr import Sdf
import omni.timeline
import omni.usd


BASE_DRIVE_PROFILE = {
    "xy_stiffness": 1000.0,
    "xy_damping": 100.0,
    "xy_max_force": 3000.0,
    "xy_max_velocity": 2.0,
    "z_stiffness": 1000.0,
    "z_damping": 100.0,
    "z_max_force": 3000.0,
    "z_max_velocity": 114.6,
}


def find_robot_root(stage):
    for robot_root in ("/World/autolife", "/autolife"):
        if stage.GetPrimAtPath(f"{robot_root}/joints/Joint_Ground_Vehicle_X").IsValid():
            return robot_root

    for prim in stage.Traverse():
        if prim.GetName() == "Joint_Ground_Vehicle_X":
            path = str(prim.GetPath())
            suffix = "/joints/Joint_Ground_Vehicle_X"
            if path.endswith(suffix):
                return path[: -len(suffix)]

    raise RuntimeError("Cannot find Joint_Ground_Vehicle_X under the current stage.")


def set_attr(prim, attr_name, value, value_type=Sdf.ValueTypeNames.Double):
    attr = prim.GetAttribute(attr_name)
    if not attr:
        attr = prim.CreateAttribute(attr_name, value_type)
    attr.Set(value)


def apply_linear_drive(stage, robot_root, joint_name):
    joint_path = f"{robot_root}/joints/{joint_name}"
    prim = stage.GetPrimAtPath(joint_path)
    if not prim.IsValid():
        raise RuntimeError(f"Missing joint prim: {joint_path}")

    set_attr(prim, "drive:linear:physics:stiffness", BASE_DRIVE_PROFILE["xy_stiffness"])
    set_attr(prim, "drive:linear:physics:damping", BASE_DRIVE_PROFILE["xy_damping"])
    set_attr(prim, "drive:linear:physics:maxForce", BASE_DRIVE_PROFILE["xy_max_force"])
    set_attr(prim, "physxJoint:maxJointVelocity", BASE_DRIVE_PROFILE["xy_max_velocity"])


def apply_angular_drive(stage, robot_root, joint_name):
    joint_path = f"{robot_root}/joints/{joint_name}"
    prim = stage.GetPrimAtPath(joint_path)
    if not prim.IsValid():
        raise RuntimeError(f"Missing joint prim: {joint_path}")

    set_attr(prim, "drive:angular:physics:stiffness", BASE_DRIVE_PROFILE["z_stiffness"])
    set_attr(prim, "drive:angular:physics:damping", BASE_DRIVE_PROFILE["z_damping"])
    set_attr(prim, "drive:angular:physics:maxForce", BASE_DRIVE_PROFILE["z_max_force"])
    set_attr(prim, "physxJoint:maxJointVelocity", BASE_DRIVE_PROFILE["z_max_velocity"])


def print_drive(stage, robot_root):
    rows = [
        ("Joint_Ground_Vehicle_X", "linear"),
        ("Joint_Ground_Vehicle_Y", "linear"),
        ("Joint_Ground_Vehicle_Z", "angular"),
    ]
    for joint_name, drive_axis in rows:
        prim = stage.GetPrimAtPath(f"{robot_root}/joints/{joint_name}")
        stiffness = prim.GetAttribute(f"drive:{drive_axis}:physics:stiffness").Get()
        damping = prim.GetAttribute(f"drive:{drive_axis}:physics:damping").Get()
        max_force = prim.GetAttribute(f"drive:{drive_axis}:physics:maxForce").Get()
        max_velocity = prim.GetAttribute("physxJoint:maxJointVelocity").Get()
        print(
            f"{joint_name}: stiffness={stiffness}, damping={damping}, "
            f"maxForce={max_force}, maxJointVelocity={max_velocity}"
        )


stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("No USD stage is currently open.")

timeline = omni.timeline.get_timeline_interface()
if timeline.is_playing():
    timeline.stop()

robot_root = find_robot_root(stage)
apply_linear_drive(stage, robot_root, "Joint_Ground_Vehicle_X")
apply_linear_drive(stage, robot_root, "Joint_Ground_Vehicle_Y")
apply_angular_drive(stage, robot_root, "Joint_Ground_Vehicle_Z")

print(f"Applied base drive profile to {robot_root}.")
print_drive(stage, robot_root)
print("Now press Play and run the ROS2 response test.")
