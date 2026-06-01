"""Autolife-Planning adapter used by the ROS action server."""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Any

import numpy as np
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from autolife_planning_bridge.dependency_paths import ensure_autolife_planning_importable
from autolife_planning_bridge.joint_mapping import CONTROL_JOINTS, PLANNING_JOINTS


@dataclass(frozen=True)
class PipelineOptions:
    planner_name: str
    time_limit: float
    point_radius: float
    simplify: bool
    interpolate: bool
    interpolate_count: int
    resolution: float
    sample_dt: float
    max_deviation: float
    velocity_scaling: float
    acceleration_scaling: float
    filter_self_from_pointcloud: bool


@dataclass
class PlanResult:
    group: str
    path: np.ndarray
    trajectory: JointTrajectory
    planning_time_ns: int
    path_cost: float


@dataclass
class _Runtime:
    create_planner: Any
    planner_config_type: Any
    parameterizer_type: Any
    max_velocity: np.ndarray
    max_acceleration: np.ndarray


class PlanningPipeline:
    """Small stateful wrapper around Autolife-Planning's native planner."""

    def __init__(self) -> None:
        self.runtime = self._load_runtime()
        self._planner = None
        self._planner_signature = None
        self._lock = threading.Lock()

    @staticmethod
    def _load_runtime() -> _Runtime:
        ensure_autolife_planning_importable()
        try:
            from autolife_planning.autolife import autolife_robot_config
            from autolife_planning.planning import create_planner
            from autolife_planning.trajectory import TimeOptimalParameterizer
            from autolife_planning.types import PlannerConfig
        except ImportError as exc:
            raise RuntimeError(
                "Cannot import autolife_planning. Install it into the ROS 2 Python "
                "environment first: python3 -m pip install --user -e "
                "/data/jiaxuanLin/Autolife-Planning"
            ) from exc

        if tuple(autolife_robot_config.joint_names) != PLANNING_JOINTS:
            raise RuntimeError(
                "Autolife-Planning joint order differs from autolife_planning_bridge. "
                "Update joint_mapping.py before using the bridge."
            )

        return _Runtime(
            create_planner=create_planner,
            planner_config_type=PlannerConfig,
            parameterizer_type=TimeOptimalParameterizer,
            max_velocity=np.asarray(autolife_robot_config.max_velocity, dtype=np.float64),
            max_acceleration=np.asarray(
                autolife_robot_config.max_acceleration, dtype=np.float64
            ),
        )

    def plan_to_config(
        self,
        group: str,
        start_config: np.ndarray,
        goal_config: np.ndarray,
        options: PipelineOptions,
        pointcloud: np.ndarray | None = None,
    ) -> PlanResult:
        start_config = np.asarray(start_config, dtype=np.float64)
        goal_config = np.asarray(goal_config, dtype=np.float64)
        if start_config.shape != (len(PLANNING_JOINTS),):
            raise ValueError(f"start_config must be shape ({len(PLANNING_JOINTS)},)")
        if goal_config.shape != (len(PLANNING_JOINTS),):
            raise ValueError(f"goal_config must be shape ({len(PLANNING_JOINTS)},)")

        with self._lock:
            planner = self._get_planner(group, start_config, options)
            active_start = planner.extract_config(start_config)
            active_goal = planner.extract_config(goal_config)

            self._update_pointcloud(planner, active_start, pointcloud, options)

            if np.allclose(active_start, active_goal, atol=1e-6, rtol=0.0):
                full_path = np.vstack([start_config, goal_config])
                trajectory = self._parameterize_to_trajectory(full_path, options)
                return PlanResult(group, full_path, trajectory, 0, 0.0)

            result = planner.plan(active_start, active_goal, time_limit=options.time_limit)
            if not result.success or result.path is None:
                raise RuntimeError(f"Planner returned {result.status.value}")

            full_path = planner.embed_path(result.path, base_config=start_config)
            trajectory = self._parameterize_to_trajectory(full_path, options)
            return PlanResult(
                group=group,
                path=full_path,
                trajectory=trajectory,
                planning_time_ns=result.planning_time_ns,
                path_cost=result.path_cost,
            )

    def _get_planner(
        self,
        group: str,
        base_config: np.ndarray,
        options: PipelineOptions,
    ):
        signature = (
            options.planner_name,
            options.point_radius,
            options.simplify,
            options.interpolate,
            options.interpolate_count,
            options.resolution,
        )
        if self._planner is None or self._planner_signature != signature:
            config = self.runtime.planner_config_type(
                planner_name=options.planner_name,
                time_limit=options.time_limit,
                point_radius=options.point_radius,
                simplify=options.simplify,
                interpolate=options.interpolate,
                interpolate_count=options.interpolate_count,
                resolution=options.resolution,
            )
            self._planner = self.runtime.create_planner(
                group,
                config=config,
                base_config=base_config,
            )
            self._planner_signature = signature
        else:
            self._planner.set_subgroup(group, base_config=base_config)
            self._planner.clear_constraints()
            self._planner.clear_costs()
        return self._planner

    def _update_pointcloud(
        self,
        planner,
        active_start: np.ndarray,
        pointcloud: np.ndarray | None,
        options: PipelineOptions,
    ) -> None:
        if pointcloud is None or len(pointcloud) == 0:
            if planner.has_pointcloud:
                planner.remove_pointcloud()
            return

        cloud = np.asarray(pointcloud, dtype=np.float32)
        if cloud.ndim != 2 or cloud.shape[1] != 3:
            raise ValueError(f"pointcloud must have shape (N, 3), got {cloud.shape}")
        if options.filter_self_from_pointcloud:
            cloud = planner.filter_self_from_pointcloud(
                cloud,
                options.point_radius,
                active_start,
            )
        planner.add_pointcloud(cloud)

    def _parameterize_to_trajectory(
        self,
        path: np.ndarray,
        options: PipelineOptions,
    ) -> JointTrajectory:
        path = np.asarray(path, dtype=np.float64)
        if path.ndim != 2 or path.shape[1] != len(PLANNING_JOINTS):
            raise ValueError(
                f"path must have shape (N, {len(PLANNING_JOINTS)}), got {path.shape}"
            )
        if path.shape[0] < 2 or np.allclose(path[0], path[-1], atol=1e-9, rtol=0.0):
            return _single_point_trajectory(path[-1], 0.1)

        parameterizer = self.runtime.parameterizer_type(
            self.runtime.max_velocity,
            self.runtime.max_acceleration,
            max_deviation=options.max_deviation,
        )
        trajectory = parameterizer.parameterize(
            path,
            velocity_scaling=options.velocity_scaling,
            acceleration_scaling=options.acceleration_scaling,
        )
        times, positions, velocities, _accelerations = trajectory.sample_uniform(
            options.sample_dt
        )

        ros_trajectory = JointTrajectory()
        ros_trajectory.joint_names = list(CONTROL_JOINTS)
        for time_from_start, row, velocity_row in zip(times, positions, velocities):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in row]
            point.velocities = [float(value) for value in velocity_row]
            point.time_from_start = seconds_to_duration(float(time_from_start))
            ros_trajectory.points.append(point)
        return ros_trajectory


def seconds_to_duration(seconds: float) -> Duration:
    sec = int(math.floor(seconds))
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


def _single_point_trajectory(positions: np.ndarray, duration: float) -> JointTrajectory:
    trajectory = JointTrajectory()
    trajectory.joint_names = list(CONTROL_JOINTS)
    point = JointTrajectoryPoint()
    point.positions = [float(value) for value in positions]
    point.velocities = [0.0 for _ in trajectory.joint_names]
    point.time_from_start = seconds_to_duration(duration)
    trajectory.points.append(point)
    return trajectory
