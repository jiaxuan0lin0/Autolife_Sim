import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import omni.graph.core as og

AUTOLIFE_SENSORS_SOURCE = Path(__file__).resolve().parents[2] / "autolife_sensors"
if AUTOLIFE_SENSORS_SOURCE.is_dir() and str(AUTOLIFE_SENSORS_SOURCE) not in sys.path:
    sys.path.insert(0, str(AUTOLIFE_SENSORS_SOURCE))

from autolife_sensors.sensor_config import (
    TF_MESSAGE_TYPE,
    TF_TOPIC,
    camera_optical_frame_id,
    camera_message_type,
    camera_outputs_for_role,
    camera_topic,
    frame_id_for_sensor,
    include_camera_prim,
    imu_topic,
    lidar_message_type,
    lidar_outputs_for_role,
    lidar_topic,
    sanitize_name,
    sensor_role_from_path,
    IMU_MESSAGE_TYPE,
)


def setup_sensor_bridge(
    stage,
    graph_path="/AutolifeSensorGraph",
    robot_prim_path="/World/autolife",
    robot_articulation_path="/World/autolife/root_joint",
    camera_width=640,
    camera_height=480,
    frame_skip_count=0,
    manifest_path=None,
):
    import usdrt

    sensors = discover_sensors(stage, robot_prim_path)
    _assign_unique_sensor_names(sensors["cameras"])
    _assign_unique_sensor_names(sensors["imus"])
    _assign_unique_sensor_names(sensors["lidars"])
    _add_camera_optical_frames(stage, sensors["cameras"])

    create_nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("RosContext", "isaacsim.ros2.bridge.ROS2Context"),
    ]
    set_values = []
    connections = []

    tf_targets = _tf_targets(sensors, robot_articulation_path)
    if tf_targets:
        create_nodes.append(("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"))
        set_values.extend(
            [
                ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path(path) for path in tf_targets]),
                ("PublishTF.inputs:topicName", TF_TOPIC),
                ("PublishTF.inputs:queueSize", 10),
            ]
        )
        if stage.GetPrimAtPath("/World").IsValid():
            set_values.append(("PublishTF.inputs:parentPrim", [usdrt.Sdf.Path("/World")]))
        connections.extend(
            [
                ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ("RosContext.outputs:context", "PublishTF.inputs:context"),
            ]
        )

    camera_output_count = 0
    for camera in sensors["cameras"]:
        sensor_name = camera["sensor_name"]
        frame_id = _record_frame_id(camera)
        node_suffix = _node_suffix(sensor_name)
        render_node = f"CreateRenderProduct_{node_suffix}"
        camera_path = camera["path"]

        create_nodes.append((render_node, "isaacsim.core.nodes.IsaacCreateRenderProduct"))
        set_values.extend(
            [
                (f"{render_node}.inputs:cameraPrim", [usdrt.Sdf.Path(camera_path)]),
                (f"{render_node}.inputs:width", int(camera_width)),
                (f"{render_node}.inputs:height", int(camera_height)),
            ]
        )
        connections.append(("OnPlaybackTick.outputs:tick", f"{render_node}.inputs:execIn"))

        for output_type in camera_outputs_for_role(camera["role"]):
            if output_type == "camera_info":
                node_name = f"CameraInfo_{node_suffix}"
                create_nodes.append((node_name, "isaacsim.ros2.bridge.ROS2CameraInfoHelper"))
                set_values.extend(
                    [
                        (f"{node_name}.inputs:topicName", camera_topic(sensor_name, output_type)),
                        (f"{node_name}.inputs:frameId", frame_id),
                        (f"{node_name}.inputs:frameSkipCount", int(frame_skip_count)),
                        (f"{node_name}.inputs:queueSize", 10),
                    ]
                )
                connections.extend(
                    [
                        (f"{render_node}.outputs:execOut", f"{node_name}.inputs:execIn"),
                        (
                            f"{render_node}.outputs:renderProductPath",
                            f"{node_name}.inputs:renderProductPath",
                        ),
                        ("RosContext.outputs:context", f"{node_name}.inputs:context"),
                    ]
                )
            else:
                node_name = f"Camera_{output_type}_{node_suffix}"
                create_nodes.append((node_name, "isaacsim.ros2.bridge.ROS2CameraHelper"))
                set_values.extend(
                    [
                        (f"{node_name}.inputs:topicName", camera_topic(sensor_name, output_type)),
                        (f"{node_name}.inputs:type", output_type),
                        (f"{node_name}.inputs:frameId", frame_id),
                        (f"{node_name}.inputs:frameSkipCount", int(frame_skip_count)),
                        (f"{node_name}.inputs:queueSize", 10),
                    ]
                )
                connections.extend(
                    [
                        (f"{render_node}.outputs:execOut", f"{node_name}.inputs:execIn"),
                        (
                            f"{render_node}.outputs:renderProductPath",
                            f"{node_name}.inputs:renderProductPath",
                        ),
                        ("RosContext.outputs:context", f"{node_name}.inputs:context"),
                    ]
                )
            camera_output_count += 1

    imu_output_count = 0
    for imu in sensors["imus"]:
        sensor_name = imu["sensor_name"]
        node_suffix = _node_suffix(sensor_name)
        read_node = f"ReadIMU_{node_suffix}"
        publish_node = f"PublishIMU_{node_suffix}"

        create_nodes.extend(
            [
                (read_node, "isaacsim.sensors.physics.IsaacReadIMU"),
                (publish_node, "isaacsim.ros2.bridge.ROS2PublishImu"),
            ]
        )
        set_values.extend(
            [
                (f"{read_node}.inputs:imuPrim", [usdrt.Sdf.Path(imu["path"])]),
                (f"{read_node}.inputs:readGravity", True),
                (f"{publish_node}.inputs:topicName", imu_topic(sensor_name)),
                (f"{publish_node}.inputs:frameId", frame_id_for_sensor(sensor_name)),
                (f"{publish_node}.inputs:queueSize", 10),
            ]
        )
        connections.extend(
            [
                ("OnPlaybackTick.outputs:tick", f"{read_node}.inputs:execIn"),
                (f"{read_node}.outputs:execOut", f"{publish_node}.inputs:execIn"),
                (f"{read_node}.outputs:orientation", f"{publish_node}.inputs:orientation"),
                (f"{read_node}.outputs:angVel", f"{publish_node}.inputs:angularVelocity"),
                (f"{read_node}.outputs:linAcc", f"{publish_node}.inputs:linearAcceleration"),
                ("ReadSimTime.outputs:simulationTime", f"{publish_node}.inputs:timeStamp"),
                ("RosContext.outputs:context", f"{publish_node}.inputs:context"),
            ]
        )
        imu_output_count += 1

    lidar_output_count = 0
    for lidar in sensors["lidars"]:
        sensor_name = lidar["sensor_name"]
        node_suffix = _node_suffix(sensor_name)
        render_node = f"CreateLidarRenderProduct_{node_suffix}"

        create_nodes.append((render_node, "isaacsim.core.nodes.IsaacCreateRenderProduct"))
        set_values.append((f"{render_node}.inputs:cameraPrim", [usdrt.Sdf.Path(lidar["path"])]))
        connections.append(("OnPlaybackTick.outputs:tick", f"{render_node}.inputs:execIn"))

        for output_type in lidar_outputs_for_role(lidar["role"]):
            node_name = f"Lidar_{output_type}_{node_suffix}"
            create_nodes.append((node_name, "isaacsim.ros2.bridge.ROS2RtxLidarHelper"))
            set_values.extend(
                [
                    (f"{node_name}.inputs:topicName", lidar_topic(sensor_name, output_type)),
                    (f"{node_name}.inputs:type", output_type),
                    (f"{node_name}.inputs:frameId", frame_id_for_sensor(sensor_name)),
                    (f"{node_name}.inputs:frameSkipCount", int(frame_skip_count)),
                    (f"{node_name}.inputs:queueSize", 10),
                ]
            )
            connections.extend(
                [
                    (f"{render_node}.outputs:execOut", f"{node_name}.inputs:execIn"),
                    (
                        f"{render_node}.outputs:renderProductPath",
                        f"{node_name}.inputs:renderProductPath",
                    ),
                    ("RosContext.outputs:context", f"{node_name}.inputs:context"),
                ]
            )
            lidar_output_count += 1

    if len(create_nodes) == 3:
        _write_manifest(
            manifest_path=manifest_path,
            graph_path=graph_path,
            robot_prim_path=robot_prim_path,
            robot_articulation_path=robot_articulation_path,
            camera_width=camera_width,
            camera_height=camera_height,
            frame_skip_count=frame_skip_count,
            sensors=sensors,
            tf_targets=tf_targets,
        )
        return {
            "cameras": 0,
            "camera_outputs": 0,
            "imus": 0,
            "imu_outputs": 0,
            "lidars": 0,
            "lidar_outputs": 0,
            "tf_targets": 0,
        }

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: create_nodes,
            og.Controller.Keys.SET_VALUES: set_values,
            og.Controller.Keys.CONNECT: connections,
        },
    )

    _write_manifest(
        manifest_path=manifest_path,
        graph_path=graph_path,
        robot_prim_path=robot_prim_path,
        robot_articulation_path=robot_articulation_path,
        camera_width=camera_width,
        camera_height=camera_height,
        frame_skip_count=frame_skip_count,
        sensors=sensors,
        tf_targets=tf_targets,
    )

    return {
        "cameras": len(sensors["cameras"]),
        "camera_outputs": camera_output_count,
        "imus": len(sensors["imus"]),
        "imu_outputs": imu_output_count,
        "lidars": len(sensors["lidars"]),
        "lidar_outputs": lidar_output_count,
        "tf_targets": len(tf_targets),
    }


def _add_camera_optical_frames(stage, cameras):
    from pxr import UsdGeom

    for camera in cameras:
        frame_id = camera_optical_frame_id(camera["sensor_name"])
        frame_path = f"{camera['path']}/{frame_id}"
        UsdGeom.Camera.Define(stage, frame_path)
        camera["frame_id"] = frame_id
        camera["frame_path"] = frame_path


def _write_manifest(
    manifest_path,
    graph_path,
    robot_prim_path,
    robot_articulation_path,
    camera_width,
    camera_height,
    frame_skip_count,
    sensors,
    tf_targets,
):
    if manifest_path is None:
        return

    path = Path(manifest_path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    manifest = _build_manifest(
        graph_path=graph_path,
        robot_prim_path=robot_prim_path,
        robot_articulation_path=robot_articulation_path,
        camera_width=camera_width,
        camera_height=camera_height,
        frame_skip_count=frame_skip_count,
        sensors=sensors,
        tf_targets=tf_targets,
    )
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")


def _build_manifest(
    graph_path,
    robot_prim_path,
    robot_articulation_path,
    camera_width,
    camera_height,
    frame_skip_count,
    sensors,
    tf_targets,
):
    return {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "graph_path": graph_path,
        "robot_prim_path": robot_prim_path,
        "robot_articulation_path": robot_articulation_path,
        "camera_width": int(camera_width),
        "camera_height": int(camera_height),
        "frame_skip_count": int(frame_skip_count),
        "tf": {
            "topic": TF_TOPIC,
            "type": TF_MESSAGE_TYPE,
            "target_prims": list(tf_targets),
            "frame_ids": _frame_ids_for_records(sensors),
        },
        "sensors": (
            [_camera_manifest(record) for record in sensors["cameras"]]
            + [_imu_manifest(record) for record in sensors["imus"]]
            + [_lidar_manifest(record) for record in sensors["lidars"]]
        ),
    }


def _camera_manifest(record):
    sensor_name = record["sensor_name"]
    frame_id = _record_frame_id(record)
    return {
        "kind": "camera",
        "name": sensor_name,
        "role": record["role"],
        "prim_path": record["path"],
        "frame_id": frame_id,
        "frame_prim_path": record.get("frame_path", record["path"]),
        "outputs": [
            {
                "name": output_type,
                "topic": camera_topic(sensor_name, output_type),
                "type": camera_message_type(output_type),
                "frame_id": frame_id,
            }
            for output_type in camera_outputs_for_role(record["role"])
        ],
    }


def _imu_manifest(record):
    sensor_name = record["sensor_name"]
    return {
        "kind": "imu",
        "name": sensor_name,
        "role": record["role"],
        "prim_path": record["path"],
        "frame_id": frame_id_for_sensor(sensor_name),
        "outputs": [
            {
                "name": "imu",
                "topic": imu_topic(sensor_name),
                "type": IMU_MESSAGE_TYPE,
                "frame_id": frame_id_for_sensor(sensor_name),
            }
        ],
    }


def _lidar_manifest(record):
    sensor_name = record["sensor_name"]
    return {
        "kind": "lidar",
        "name": sensor_name,
        "role": record["role"],
        "prim_path": record["path"],
        "frame_id": frame_id_for_sensor(sensor_name),
        "outputs": [
            {
                "name": output_type,
                "topic": lidar_topic(sensor_name, output_type),
                "type": lidar_message_type(output_type),
                "frame_id": frame_id_for_sensor(sensor_name),
            }
            for output_type in lidar_outputs_for_role(record["role"])
        ],
    }


def _frame_ids_for_records(sensors):
    frame_ids = []
    for group_name in ("cameras", "imus", "lidars"):
        for record in sensors[group_name]:
            frame_ids.append(_record_frame_id(record))
    return _dedupe(frame_ids)


def _record_frame_id(record):
    return record.get("frame_id", frame_id_for_sensor(record["sensor_name"]))


def discover_sensors(stage, robot_prim_path):
    from pxr import Usd

    root = stage.GetPrimAtPath(robot_prim_path)
    if not root.IsValid():
        return {"cameras": [], "imus": [], "lidars": []}

    sensors = {"cameras": [], "imus": [], "lidars": []}
    predicate = Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)
    for prim in Usd.PrimRange.Stage(stage, predicate):
        prim_path = str(prim.GetPath())
        if prim_path != robot_prim_path and not prim_path.startswith(f"{robot_prim_path}/"):
            continue

        type_name = prim.GetTypeName()
        if type_name == "Camera":
            role = sensor_role_from_path(prim_path, "camera")
            if not include_camera_prim(role, prim_path):
                continue
            sensors["cameras"].append(_sensor_record(prim_path, role, _camera_priority(prim_path)))
        elif type_name == "IsaacImuSensor":
            role = sensor_role_from_path(prim_path, "imu")
            sensors["imus"].append(_sensor_record(prim_path, role, 0))
        elif type_name == "OmniLidar":
            role = sensor_role_from_path(prim_path, "lidar")
            sensors["lidars"].append(_sensor_record(prim_path, role, 0))

    for records in sensors.values():
        records.sort(key=lambda item: (item["role"], item["priority"], item["path"]))

    return sensors


def _sensor_record(path, role, priority):
    return {
        "path": path,
        "role": role,
        "sensor_name": role,
        "priority": priority,
    }


def _assign_unique_sensor_names(records):
    seen = {}
    for record in records:
        base_name = record["role"]
        count = seen.get(base_name, 0)
        if count == 0:
            record["sensor_name"] = base_name
        else:
            leaf_name = record["path"].rsplit("/", 1)[-1]
            record["sensor_name"] = f"{base_name}_{sanitize_name(leaf_name)}"
        seen[base_name] = count + 1


def _tf_targets(sensors, robot_articulation_path):
    targets = [robot_articulation_path]
    targets.extend(
        record.get("frame_path", record["path"])
        for record in sensors["cameras"]
    )
    for group_name in ("imus", "lidars"):
        targets.extend(record["path"] for record in sensors[group_name])
    return _dedupe(targets)


def _dedupe(values):
    seen = set()
    result = []
    for value in values:
        if value in seen:
            continue
        seen.add(value)
        result.append(value)
    return result


def _camera_priority(path):
    lowered = path.lower()
    if "rgb" in lowered or "color" in lowered:
        return 0
    if "depth" in lowered:
        return 1
    return 2


def _node_suffix(sensor_name):
    sanitized = sanitize_name(sensor_name)
    return sanitized[0].upper() + sanitized[1:]
