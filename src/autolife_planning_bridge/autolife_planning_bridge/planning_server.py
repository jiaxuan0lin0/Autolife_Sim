"""ROS action server for Autolife planning and execution."""

from __future__ import annotations

import math
import threading
import time
import traceback

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from autolife_planning_msgs.action import JointControl, PoseControl, TrajectoryExecution
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from rclpy._rclpy_pybind11 import RCLError
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration as RclpyDuration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory

from autolife_planning_bridge.geometry import (
    pose_to_arrays,
    quat_to_matrix_xyzw,
    transform_points,
    transform_pose,
)
from autolife_planning_bridge.dependency_paths import ensure_autolife_planning_importable
from autolife_planning_bridge.joint_mapping import (
    CONTROL_JOINT_SET,
    PLANNING_JOINTS,
    UNPLANNED_CONTROL_JOINTS,
    chain_factory_args,
    chain_joint_names,
    control_to_planning_names,
    default_group_for_chain,
    extract_positions,
    merge_goal_positions,
    planning_to_control_names,
    resolve_chain_key,
    resolve_group,
    state_to_planning_vector,
    validate_group,
)
from autolife_planning_bridge.planning_pipeline import PlanningPipeline, PipelineOptions


class PlanningServer(Node):
    def __init__(self):
        super().__init__("autolife_planning_server")
        self.callback_group = ReentrantCallbackGroup()

        self.planning_frame = self.declare_parameter("planning_frame", "World").value
        self.joint_states_topic = self.declare_parameter(
            "joint_states_topic", "/joint_states"
        ).value
        self.controller_action = self.declare_parameter(
            "controller_action", "/whole_body_controller/follow_joint_trajectory"
        ).value
        self.joint_control_action = self.declare_parameter(
            "joint_control_action", "/autolife_planning/joint_control"
        ).value
        self.pose_control_action = self.declare_parameter(
            "pose_control_action", "/autolife_planning/pose_control"
        ).value
        self.trajectory_execution_action = self.declare_parameter(
            "trajectory_execution_action", "/autolife_planning/trajectory_execution"
        ).value

        self.default_planner_name = self.declare_parameter("planner_name", "bitstar").value
        self.default_time_limit = float(self.declare_parameter("time_limit", 2.0).value)
        self.point_radius = float(self.declare_parameter("point_radius", 0.01).value)
        self.simplify = bool(self.declare_parameter("simplify", True).value)
        self.interpolate = bool(self.declare_parameter("interpolate", True).value)
        self.interpolate_count = int(self.declare_parameter("interpolate_count", 0).value)
        self.resolution = float(self.declare_parameter("resolution", 64.0).value)
        self.default_sample_dt = float(self.declare_parameter("sample_dt", 0.02).value)
        self.max_deviation = float(self.declare_parameter("max_deviation", 0.1).value)
        self.default_velocity_scaling = float(
            self.declare_parameter("velocity_scaling", 1.0).value
        )
        self.default_acceleration_scaling = float(
            self.declare_parameter("acceleration_scaling", 1.0).value
        )
        self.initial_state_timeout = float(
            self.declare_parameter("initial_state_timeout", 3.0).value
        )
        self.controller_server_timeout = float(
            self.declare_parameter("controller_server_timeout", 5.0).value
        )

        self.default_ik_backend = self.declare_parameter("ik_backend", "trac_ik").value
        self.ik_self_collision = bool(
            self.declare_parameter("ik_self_collision", False).value
        )
        self.ik_timeout = float(self.declare_parameter("ik_timeout", 0.2).value)
        self.ik_max_attempts = int(self.declare_parameter("ik_max_attempts", 10).value)
        self.ik_position_tolerance = float(
            self.declare_parameter("ik_position_tolerance", 1e-4).value
        )
        self.ik_orientation_tolerance = float(
            self.declare_parameter("ik_orientation_tolerance", 1e-4).value
        )

        self.pointcloud_topic = self.declare_parameter("pointcloud_topic", "").value
        self.pointcloud_max_points = int(
            self.declare_parameter("pointcloud_max_points", 20000).value
        )
        self.filter_self_from_pointcloud = bool(
            self.declare_parameter("filter_self_from_pointcloud", True).value
        )

        self._validate_parameters()

        self.positions_by_control_name: dict[str, float] = {}
        self.state_lock = threading.Lock()
        self.goal_lock = threading.Lock()
        self.latest_pointcloud: np.ndarray | None = None
        self.pointcloud_lock = threading.Lock()
        self.ik_solvers = {}
        self.ik_lock = threading.Lock()

        self.pipeline = PlanningPipeline()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._joint_states_cb,
            20,
            callback_group=self.callback_group,
        )
        if self.pointcloud_topic:
            self.create_subscription(
                PointCloud2,
                self.pointcloud_topic,
                self._pointcloud_cb,
                1,
                callback_group=self.callback_group,
            )

        self.controller_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.controller_action,
            callback_group=self.callback_group,
        )
        self.joint_server = ActionServer(
            self,
            JointControl,
            self.joint_control_action,
            self.execute_joint_control,
            callback_group=self.callback_group,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
        )
        self.pose_server = ActionServer(
            self,
            PoseControl,
            self.pose_control_action,
            self.execute_pose_control,
            callback_group=self.callback_group,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
        )
        self.execution_server = ActionServer(
            self,
            TrajectoryExecution,
            self.trajectory_execution_action,
            self.execute_trajectory_execution,
            callback_group=self.callback_group,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
        )

        self.get_logger().info(
            "Autolife planning actions ready: "
            f"{self.joint_control_action}, {self.pose_control_action}, "
            f"{self.trajectory_execution_action}"
        )

    def _validate_parameters(self) -> None:
        if self.default_time_limit <= 0.0:
            raise ValueError("time_limit must be positive")
        if self.point_radius <= 0.0:
            raise ValueError("point_radius must be positive")
        if self.default_sample_dt <= 0.0:
            raise ValueError("sample_dt must be positive")
        if self.max_deviation <= 0.0:
            raise ValueError("max_deviation must be positive")
        if not (0.0 < self.default_velocity_scaling <= 1.0):
            raise ValueError("velocity_scaling must be in (0, 1]")
        if not (0.0 < self.default_acceleration_scaling <= 1.0):
            raise ValueError("acceleration_scaling must be in (0, 1]")
        if self.pointcloud_max_points < 1:
            raise ValueError("pointcloud_max_points must be >= 1")

    def _joint_states_cb(self, msg: JointState) -> None:
        with self.state_lock:
            for name, position in zip(msg.name, msg.position):
                if name in CONTROL_JOINT_SET and math.isfinite(position):
                    self.positions_by_control_name[name] = float(position)

    def _pointcloud_cb(self, msg: PointCloud2) -> None:
        try:
            from sensor_msgs_py import point_cloud2

            points = []
            for point in point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            ):
                points.append([float(point[0]), float(point[1]), float(point[2])])
            if not points:
                return
            cloud = np.asarray(points, dtype=np.float32)
            if len(cloud) > self.pointcloud_max_points:
                indices = np.linspace(
                    0,
                    len(cloud) - 1,
                    num=self.pointcloud_max_points,
                    dtype=np.int64,
                )
                cloud = cloud[indices]
            frame_id = msg.header.frame_id
            if frame_id and frame_id != self.planning_frame:
                transform = self.tf_buffer.lookup_transform(
                    self.planning_frame,
                    frame_id,
                    Time(),
                    timeout=RclpyDuration(seconds=0.1),
                )
                cloud = transform_points(cloud, transform)
            with self.pointcloud_lock:
                self.latest_pointcloud = cloud
        except Exception as exc:  # noqa: BLE001 - sensor input should not kill node.
            self.get_logger().warn(f"Dropping pointcloud obstacle update: {exc}")

    def execute_joint_control(self, goal_handle):
        if not self._try_begin_goal(goal_handle, JointControl.Result):
            return self._make_result(
                JointControl.Result,
                False,
                "Another planning goal is already active",
            )
        try:
            return self._execute_joint_control(goal_handle)
        finally:
            self.goal_lock.release()

    def _execute_joint_control(self, goal_handle):
        try:
            request = goal_handle.request
            self._publish_feedback(JointControl.Feedback, goal_handle, "initializing", 0.05)
            planning_names, positions = self._parse_joint_goal(
                request.joint_names,
                request.positions,
            )
            start_config = self._wait_for_start_config()
            if start_config is None:
                raise RuntimeError("Timed out waiting for complete /joint_states")

            goal_config = merge_goal_positions(start_config, planning_names, positions)
            group = resolve_group(request.group, planning_names)
            options = self._options_from_request(request)
            pointcloud = self._get_latest_pointcloud()

            self._publish_feedback(
                JointControl.Feedback,
                goal_handle,
                "planning",
                0.25,
                f"group={group}",
            )
            plan = self.pipeline.plan_to_config(
                group,
                start_config,
                goal_config,
                options,
                pointcloud=pointcloud,
            )
            if request.plan_only:
                goal_handle.succeed()
                return self._make_result(
                    JointControl.Result,
                    True,
                    f"Planned {len(plan.trajectory.points)} trajectory points",
                    trajectory=plan.trajectory,
                )

            self._publish_feedback(JointControl.Feedback, goal_handle, "executing", 0.7)
            return self._execute_planned_trajectory(
                goal_handle,
                plan.trajectory,
                JointControl.Result,
                JointControl.Feedback,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"JointControl failed: {exc}")
            goal_handle.abort()
            return self._make_result(JointControl.Result, False, str(exc))

    def execute_pose_control(self, goal_handle):
        if not self._try_begin_goal(goal_handle, PoseControl.Result):
            return self._make_result(
                PoseControl.Result,
                False,
                "Another planning goal is already active",
            )
        try:
            return self._execute_pose_control(goal_handle)
        finally:
            self.goal_lock.release()

    def _execute_pose_control(self, goal_handle):
        try:
            request = goal_handle.request
            self._publish_feedback(PoseControl.Feedback, goal_handle, "initializing", 0.05)
            start_config = self._wait_for_start_config()
            if start_config is None:
                raise RuntimeError("Timed out waiting for complete /joint_states")

            target_pose = self._pose_in_planning_frame(request.target_pose)
            chain_key = resolve_chain_key(request.chain, request.side)
            chain_joints = chain_joint_names(chain_key)
            ik_solution = self._solve_ik(request, chain_key, target_pose.pose, start_config)
            goal_config = merge_goal_positions(start_config, chain_joints, ik_solution)

            group = request.group
            if group in ("", "auto"):
                group = default_group_for_chain(chain_key)
            validate_group(group, chain_joints)
            options = self._options_from_request(request)
            pointcloud = self._get_latest_pointcloud()

            self._publish_feedback(
                PoseControl.Feedback,
                goal_handle,
                "planning",
                0.35,
                f"group={group}, chain={chain_key}",
            )
            plan = self.pipeline.plan_to_config(
                group,
                start_config,
                goal_config,
                options,
                pointcloud=pointcloud,
            )
            joint_goal = self._joint_goal_msg(chain_joints, ik_solution)
            if request.plan_only:
                goal_handle.succeed()
                return self._make_result(
                    PoseControl.Result,
                    True,
                    f"IK and planning succeeded; {len(plan.trajectory.points)} trajectory points",
                    trajectory=plan.trajectory,
                    joint_goal=joint_goal,
                )

            self._publish_feedback(PoseControl.Feedback, goal_handle, "executing", 0.7)
            return self._execute_planned_trajectory(
                goal_handle,
                plan.trajectory,
                PoseControl.Result,
                PoseControl.Feedback,
                joint_goal=joint_goal,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"PoseControl failed: {exc}")
            self.get_logger().debug(traceback.format_exc())
            goal_handle.abort()
            return self._make_result(PoseControl.Result, False, str(exc))

    def execute_trajectory_execution(self, goal_handle):
        if not self._try_begin_goal(goal_handle, TrajectoryExecution.Result):
            return self._make_result(
                TrajectoryExecution.Result,
                False,
                "Another planning or execution goal is already active",
            )
        try:
            trajectory = goal_handle.request.trajectory
            return self._execute_planned_trajectory(
                goal_handle,
                trajectory,
                TrajectoryExecution.Result,
                TrajectoryExecution.Feedback,
            )
        finally:
            self.goal_lock.release()

    def _try_begin_goal(self, goal_handle, result_type) -> bool:
        if self.goal_lock.acquire(blocking=False):
            return True
        goal_handle.abort()
        result = result_type()
        result.success = False
        result.message = "Another planning goal is already active"
        return False

    def _parse_joint_goal(
        self,
        control_names: list[str],
        positions: list[float],
    ) -> tuple[list[str], list[float]]:
        if not control_names:
            raise ValueError("joint_names is empty")
        if len(set(control_names)) != len(control_names):
            raise ValueError("joint_names contains duplicates")
        if len(positions) != len(control_names):
            raise ValueError(
                f"positions length {len(positions)} does not match joint_names length "
                f"{len(control_names)}"
            )
        gripper_joints = sorted(set(control_names) & UNPLANNED_CONTROL_JOINTS)
        if gripper_joints:
            raise ValueError(
                "Autolife-Planning does not plan gripper joints; command them through "
                f"the gripper controller: {gripper_joints}"
            )
        if not all(math.isfinite(value) for value in positions):
            raise ValueError("positions contains non-finite values")

        planning_names = control_to_planning_names(control_names)
        unknown = [
            control
            for control, planning in zip(control_names, planning_names)
            if planning not in PLANNING_JOINTS
        ]
        if unknown:
            raise ValueError(f"Unknown or unsupported joints: {unknown}")
        return planning_names, list(positions)

    def _wait_for_start_config(self) -> np.ndarray | None:
        deadline = time.monotonic() + self.initial_state_timeout
        last_missing = []
        while time.monotonic() <= deadline:
            with self.state_lock:
                config, missing = state_to_planning_vector(self.positions_by_control_name)
            if config is not None:
                return config
            last_missing = missing
            time.sleep(0.02)
        if last_missing:
            self.get_logger().warn(f"Missing joint states: {last_missing}")
        return None

    def _options_from_request(self, request) -> PipelineOptions:
        planner_name = request.planner_name or self.default_planner_name
        time_limit = request.time_limit if request.time_limit > 0.0 else self.default_time_limit
        velocity_scaling = (
            request.velocity_scaling
            if request.velocity_scaling > 0.0
            else self.default_velocity_scaling
        )
        acceleration_scaling = (
            request.acceleration_scaling
            if request.acceleration_scaling > 0.0
            else self.default_acceleration_scaling
        )
        sample_dt = request.sample_dt if request.sample_dt > 0.0 else self.default_sample_dt
        if not (0.0 < velocity_scaling <= 1.0):
            raise ValueError("velocity_scaling must be in (0, 1]")
        if not (0.0 < acceleration_scaling <= 1.0):
            raise ValueError("acceleration_scaling must be in (0, 1]")
        return PipelineOptions(
            planner_name=planner_name,
            time_limit=time_limit,
            point_radius=self.point_radius,
            simplify=self.simplify,
            interpolate=self.interpolate,
            interpolate_count=self.interpolate_count,
            resolution=self.resolution,
            sample_dt=sample_dt,
            max_deviation=self.max_deviation,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=acceleration_scaling,
            filter_self_from_pointcloud=self.filter_self_from_pointcloud,
        )

    def _pose_in_planning_frame(self, pose: PoseStamped) -> PoseStamped:
        frame_id = pose.header.frame_id
        if frame_id in ("", self.planning_frame):
            return pose
        transform = self.tf_buffer.lookup_transform(
            self.planning_frame,
            frame_id,
            Time(),
            timeout=RclpyDuration(seconds=0.5),
        )
        return transform_pose(pose, transform)

    def _solve_ik(
        self,
        request,
        chain_key: str,
        target_pose: Pose,
        start_config: np.ndarray,
    ) -> np.ndarray:
        ensure_autolife_planning_importable()
        from autolife_planning.types import IKConfig, PinkIKConfig, SE3Pose

        position, quat = pose_to_arrays(target_pose)
        target = SE3Pose(position=position, rotation=quat_to_matrix_xyzw(quat))
        chain_name, side = chain_factory_args(chain_key)
        backend = request.ik_backend or self.default_ik_backend
        chain_joints = chain_joint_names(chain_key)
        seed = np.asarray(extract_positions(start_config, chain_joints), dtype=np.float64)

        with self.ik_lock:
            key = (chain_key, backend, self.ik_self_collision)
            solver = self.ik_solvers.get(key)
            if solver is None:
                from autolife_planning.kinematics import create_ik_solver

                if backend == "pink":
                    config = PinkIKConfig()
                else:
                    config = IKConfig(
                        timeout=self.ik_timeout,
                        max_attempts=self.ik_max_attempts,
                        position_tolerance=self.ik_position_tolerance,
                        orientation_tolerance=self.ik_orientation_tolerance,
                    )
                solver = create_ik_solver(
                    chain_name,
                    side=side,
                    backend=backend,
                    config=config,
                    self_collision=self.ik_self_collision,
                )
                self.ik_solvers[key] = solver

        result = solver.solve(target, seed=seed)
        if not result.success or result.joint_positions is None:
            raise RuntimeError(
                f"IK failed for chain={chain_key}, backend={backend}: "
                f"{result.status.value}"
            )
        return np.asarray(result.joint_positions, dtype=np.float64)

    def _joint_goal_msg(
        self,
        planning_joint_names: tuple[str, ...],
        positions: np.ndarray,
    ) -> JointState:
        msg = JointState()
        msg.name = planning_to_control_names(planning_joint_names)
        msg.position = [float(value) for value in positions]
        return msg

    def _get_latest_pointcloud(self) -> np.ndarray | None:
        with self.pointcloud_lock:
            if self.latest_pointcloud is None:
                return None
            return self.latest_pointcloud.copy()

    def _execute_planned_trajectory(
        self,
        goal_handle,
        trajectory: JointTrajectory,
        result_type,
        feedback_type,
        **extra_fields,
    ):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._make_result(result_type, False, "Goal canceled", **extra_fields)
        if not self.controller_client.wait_for_server(
            timeout_sec=self.controller_server_timeout
        ):
            goal_handle.abort()
            return self._make_result(
                result_type,
                False,
                f"Controller action is unavailable: {self.controller_action}",
                trajectory=trajectory if hasattr(result_type(), "trajectory") else None,
                **extra_fields,
            )

        downstream_goal = FollowJointTrajectory.Goal()
        downstream_goal.trajectory = trajectory
        send_future = self.controller_client.send_goal_async(downstream_goal)
        downstream_handle = self._wait_for_future(goal_handle, send_future)
        if downstream_handle is None:
            goal_handle.canceled()
            return self._make_result(result_type, False, "Goal canceled", **extra_fields)
        if not downstream_handle.accepted:
            goal_handle.abort()
            return self._make_result(
                result_type,
                False,
                "Controller rejected trajectory",
                trajectory=trajectory if hasattr(result_type(), "trajectory") else None,
                **extra_fields,
            )

        self._publish_feedback(feedback_type, goal_handle, "executing", 0.75)
        result_future = downstream_handle.get_result_async()
        downstream_result = self._wait_for_future(goal_handle, result_future)
        if downstream_result is None:
            cancel_future = downstream_handle.cancel_goal_async()
            self._wait_for_future(goal_handle, cancel_future, allow_cancel=False)
            goal_handle.canceled()
            return self._make_result(result_type, False, "Goal canceled", **extra_fields)

        controller_result = downstream_result.result
        success = downstream_result.status == GoalStatus.STATUS_SUCCEEDED
        message = controller_result.error_string
        if success:
            goal_handle.succeed()
            if not message:
                message = "Executed trajectory"
        else:
            goal_handle.abort()
            if not message:
                message = f"Controller finished with status {downstream_result.status}"
        return self._make_result(
            result_type,
            success,
            message,
            trajectory=trajectory if hasattr(result_type(), "trajectory") else None,
            **extra_fields,
        )

    def _wait_for_future(self, goal_handle, future, allow_cancel: bool = True):
        while rclpy.ok() and not future.done():
            if allow_cancel and goal_handle.is_cancel_requested:
                return None
            time.sleep(0.01)
        if not future.done():
            return None
        return future.result()

    def _publish_feedback(
        self,
        feedback_type,
        goal_handle,
        stage: str,
        progress: float,
        message: str = "",
    ) -> None:
        feedback = feedback_type()
        feedback.stage = stage
        feedback.progress = float(progress)
        feedback.message = message
        goal_handle.publish_feedback(feedback)

    def _make_result(
        self,
        result_type,
        success: bool,
        message: str,
        trajectory: JointTrajectory | None = None,
        joint_goal: JointState | None = None,
    ):
        result = result_type()
        result.success = bool(success)
        result.message = message
        if trajectory is not None and hasattr(result, "trajectory"):
            result.trajectory = trajectory
        if joint_goal is not None and hasattr(result, "joint_goal"):
            result.joint_goal = joint_goal
        return result


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PlanningServer()
    except (Exception, TransformException):
        if rclpy.ok():
            rclpy.shutdown()
        raise

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        try:
            executor.remove_node(node)
        except RCLError:
            pass
        executor.shutdown()
        try:
            node.destroy_node()
        except RCLError:
            pass
        if rclpy.ok():
            rclpy.shutdown()
