# autolife_hardware

This package contains an early `ros2_control` hardware interface for Autolife.
It is not part of the current Isaac Sim / OmniGibson simulation workflow.

The active simulation stack uses:

- `autolife_simulation` for Isaac Sim / OmniGibson scene loading and ROS 2 bridge setup.
- `autolife_control` for ROS 2 controller nodes and command muxing.
- `autolife_sensors` for sensor naming, validation, and RViz configuration.

## Current Status

`autolife_hardware` is currently disabled with:

```text
COLCON_IGNORE
```

This prevents `colcon build` from discovering and building the package by default.
The package can remain in the repository without affecting the simulation build.

## Re-enable

Remove the ignore marker when the real `ros2_control` hardware interface becomes
part of the active workflow:

```bash
rm src/autolife_hardware/COLCON_IGNORE
colcon build --packages-select autolife_hardware
```

Before re-enabling, review `CMakeLists.txt` and the hardware interface
implementation, because this package has not been validated with the current
simulation workflow.
