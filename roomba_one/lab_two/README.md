### Control the roomba with ros2_control and teleop_twist

**Objective**: In this lab exercise, you will learn how to build a simple robot model in Gazebo with two cylinder wheels and a spherical castor wheel. You will then set up the robot for differential drive control using `ros2_control` and control it using `teleop_twist_keyboard`.

#### Prerequisites

- ROS 2 Humble installed
- Gazebo simulator installed
- Understanding of URDF (Unified Robot Description Format) and basic ROS 2 concepts
- Lab one completed

#### Instructions

---

### Part 1: Spawning a Simple Robot in Gazebo

1. **Modify robot_core.xacro:**

   Modify the robot_core.xacro and add the joint and link descriptions for the left, right and castor wheels. Use the dimensions provided in the lab_one 
   description (https://github.com/CARinternal/EE5108-Digital-Twins-for-Robotics/tree/main/roomba_one/lab_one)


2. **Launch the Robot in Gazebo:**

   Rebuild your ROS2 package, and source:
   ```bash
   cd /home/ros2_ws
   colcon build --symlink-install
   # source your package
   source install/setup.bash
   ```
   Launch the simulation:
   ```bash
   cd /home/ros2_ws/
   ros2 launch roomba_one launch_sim.launch.py world:=src/worlds/roomba_world.world
   ```

### Part 2: Setting Up Differential Drive Control

1. **Configure `ros2_control`:**

   Create a control configuration file.

   ```bash
   touch config/diff_drive.yaml
   ```

   Add the following configuration for the differential drive controller:

   ```yaml
   robot_controllers:
     diff_drive_controller:
       type: diff_drive_controller/DiffDriveController
       left_wheel: base_link/left_wheel_joint
       right_wheel: base_link/right_wheel_joint
       publish_rate: 50
       command_interface: "velocity"
       velocity_rolling_window_size: 2
       pose_publish_rate: 50
   ```

2. **Adding `ros2_control`:**

   Add the following files to your project `config` folder:
   - gaz_ros2_use_sim.yaml
   - gazebo_params.yaml
   - joystick.yaml
   - my_controllers.yaml
   - twist_mux.yaml
  
   You can find these files in the lab_two `config` folder in this repository

   Update the launch file to load and start the controller:

   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node
   from launch.actions import IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from osrf_pycommon.terminal_color import ansi
   import os

   def generate_launch_description():
       use_sim_time = LaunchConfiguration('use_sim_time', default='true')

       return LaunchDescription([
           Node(
               package='gazebo_ros',
               executable='spawn_entity.py',
               arguments=['-entity', 'simple_robot', '-file', 'urdf/simple_robot.urdf'],
               output='screen'
           ),
           Node(
               package='controller_manager',
               executable='ros2_control_node',
               parameters=['config/diff_drive.yaml'],
               output={
                   'stdout': 'screen',
                   'stderr': 'screen',
               },
           ),
           Node(
               package='diff_drive_controller',
               executable='diff_drive_controller',
               parameters=['config/diff_drive.yaml'],
               remappings=[('/cmd_vel', '/cmd_vel')],
               output='screen',
           ),
       ])
   ```

4. **Launch the Robot with Controllers:**

   Run the launch file to spawn the robot along with the controllers.

   ```bash
   ros2 launch my_robot spawn_robot_launch.py
   ```

### Part 3: Controlling the Robot with `teleop_twist_keyboard`

1. **Install `teleop_twist_keyboard`:**

   If you haven't already, install the package using:

   ```bash
   sudo apt-get install ros-<ros2-distro>-teleop-twist-keyboard
   ```

2. **Control the Robot via Keyboard:**

   Open a new terminal and run the keyboard teleoperation node:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

   Use the keyboard controls to move the robot around in Gazebo. The node translates your key presses into velocity messages sent to the `/cmd_vel` topic, which will move your robot.

#### Conclusion

In this lab exercise, you successfully built a simple robot model using URDF, setup a differential drive control using `ros2_control`, and controlled the robot using keyboard teleoperation with `teleop_twist_keyboard`. This fundamental workflow provides a foundation for working with robots in ROS 2 and Gazebo for simulation and control tasks.
