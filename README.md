# bck_description — how to view the robot in RViz and Gazebo

Quick instructions to visualize `guzergah_agv.xacro` in RViz and Gazebo (ROS 2).

Prerequisites
- A ROS 2 installation (Foxy/Galactic/Humble or newer).
- `xacro`, `robot_state_publisher`, `rviz2`, and `gazebo_ros` packages installed for your ROS 2 distro.
- Source your ROS 2 workspace after building (see below).

Build (from workspace root):

```bash
# from your workspace root (where package.xml/CMakeLists.txt live)
colcon build --packages-select bck_description
source install/setup.bash
```

Run RViz (shows robot model via `robot_state_publisher`):

```bash
# launch robot_state_publisher + rviz2
ros2 launch bck_description rviz.launch.py
```

If RViz opens but you don't see the robot:
- In RViz, click `Add` → `RobotModel`. Set `Description` to `robot_description` if necessary.

Run Gazebo and spawn the robot:

```bash
ros2 launch bck_description gazebo.launch.py
```

Notes / troubleshooting
- The `xacro` file includes `$(find ...)` style includes. Make sure the referenced package(s) (e.g. `guzergah_description`) are findable on `ROS_PACKAGE_PATH` (source your workspace or install that package).
- If `spawn_entity.py` can't find the robot, ensure `robot_state_publisher` is publishing `robot_description` parameter (check `ros2 topic echo /parameter_events` and `ros2 param get /robot_state_publisher robot_description`).
- For Gazebo 11/Classic Gazebo, `gazebo_ros` package provides the included launch. For Ignition/`ign-gazebo`, different launch/setup is required.

If you want, I can:
- Generate a pre-processed URDF file from the xacro and add it to the repo.
- Create an RViz config that automatically adds a `RobotModel` display.
