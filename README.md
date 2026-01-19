# ur5e_robotiq

ROS 2 workspace for simulating a **Universal Robots UR5e** with a **Robotiq 2F gripper** in **Gazebo (GZ Sim)**, with optional **MoveIt 2** integration.

This workspace vendors upstream packages (UR description + UR Gazebo simulation + Robotiq gripper) and adds local packages to tie them together.

## Repository Layout

Main packages you will interact with:

- `ur5e_description`
  - UR5e + Robotiq description and Gazebo launch files.
  - Provides:
    - `ur_sim_control.launch.py` (Gazebo + ros2_control + RViz)
    - `ur_sim_moveit.launch.py` (Gazebo + MoveIt + RViz)
- `ur_moveit_config`
  - MoveIt configuration used by `ur5e_description/launch/ur_sim_moveit.launch.py` by default.
- `ros2_robotiq_gripper/*`
  - Robotiq gripper description and controllers.
- `Universal_Robots_ROS2_Description/*`
  - Universal Robots description packages.
- `Universal_Robots_ROS2_GZ_Simulation/*`
  - Upstream UR Gazebo (GZ Sim) simulation launchers.

## Prerequisites

- ROS 2 installed and sourced (e.g. Humble / Iron / Jazzy)
- Gazebo (GZ Sim) + ROS-GZ bridge packages
- MoveIt 2 (optional, only needed for MoveIt launch)

If you are missing dependencies, use `rosdep` to resolve them after sourcing ROS 2.

## Build

From your workspace root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run: Gazebo + ros2_control (+ RViz)

Launch the UR5e + Robotiq simulation in GZ Sim:

```bash
source install/setup.bash
ros2 launch ur5e_description ur_sim_control.launch.py ur_type:=ur5e
```

Common launch arguments (see `ur5e_description/launch/ur_sim_control.launch.py`):

- `ur_type` (default: `ur5e`)
- `gazebo_gui` (default: `true`)
- `world_file` (default: `empty.sdf`)
- `launch_rviz` (default: `true`)
- `controllers_file` (default points to `ur5e_description/config/ur_controllers.yaml`)
- `activate_joint_controller` (default: `true`)
- `activate_gripper_controller` (default: `true`)
- `initial_joint_controller` (default: `joint_trajectory_controller`)
- `initial_gripper_controller` (default: `robotiq_gripper_controller`)

Example (headless Gazebo):

```bash
ros2 launch ur5e_description ur_sim_control.launch.py gazebo_gui:=false
```

## Run: Gazebo + MoveIt 2 (+ RViz)

This launches:

- Gazebo simulation (via `ur_simulation_gz`)
- MoveIt 2 using `ur_moveit_config`

```bash
source install/setup.bash
ros2 launch ur5e_description ur_sim_moveit.launch.py ur_type:=ur5e
```

If you have a custom MoveIt config package/launch file, you can override it:

```bash
ros2 launch ur5e_description ur_sim_moveit.launch.py \
  moveit_launch_file:=$(ros2 pkg prefix ur_moveit_config)/share/ur_moveit_config/launch/ur_moveit.launch.py
```

## Tips / Troubleshooting

- If controllers do not start, check that:
  - `ros2_control` and controller packages are installed
  - the correct `controllers_file` is being used
- If Gazebo models/resources are missing, ensure the environment is sourced:
  - `source install/setup.bash`
  - `ur5e_description/launch/ur_sim_control.launch.py` also extends `GZ_SIM_RESOURCE_PATH` at runtime
- If MoveIt launches but planning fails, verify `use_sim_time` is enabled (the MoveIt launch in this workspace sets it to `true`).

## Notes

- This workspace is intended for **simulation**. Running a real UR robot requires the UR driver stack and proper hardware setup.
