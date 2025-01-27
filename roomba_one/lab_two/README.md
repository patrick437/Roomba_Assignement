### Control the roomba with ros2_control and teleop_twist

**Objective**: In this lab exercise, you will learn how to build a simple robot model in Gazebo with two cylinder wheels and a spherical castor wheel. You will then set up the robot for differential drive control using `ros2_control` and control it using `teleop_twist_keyboard`.

#### Prerequisites

- ROS 2 Foxy or later installed
- Gazebo simulator installed
- Understanding of URDF (Unified Robot Description Format) and basic ROS 2 concepts

#### Instructions

---

### Part 1: Spawning a Simple Robot in Gazebo

1. **Create a New ROS 2 Package:**

   Open a terminal and create a new package for your robot.

   ```bash
   ros2 pkg create --build-type ament_cmake my_robot
   cd my_robot
   mkdir urdf launch config
   ```

2. **Define the Robot URDF:**

   Create a new URDF file for the robot.

   ```bash
   touch urdf/simple_robot.urdf
   ```

   Open `simple_robot.urdf` using a text editor and add the following content to define a robot base, two cylinder wheels, and a spherical castor wheel:

   ```xml
   <?xml version="1.0"?>
   <robot name="simple_robot">

     <!-- Base Link -->
     <link name="base_link">
       <visual>
         <geometry>
           <box size="0.4 0.2 0.1"/>
         </geometry>
         <origin xyz="0 0 0.05"/>
         <material name="blue">
           <color rgba="0 0 1 1"/>
         </material>
       </visual>
     </link>

     <!-- Wheel Links -->
     <link name="left_wheel">
       <visual>
         <geometry>
           <cylinder radius="0.1" length="0.05"/>
         </geometry>
         <origin xyz="-0.15 0.1 0"/>
         <material name="black">
           <color rgba="0 0 0 1"/>
         </material>
       </visual>
     </link>

     <link name="right_wheel">
       <visual>
         <geometry>
           <cylinder radius="0.1" length="0.05"/>
         </geometry>
         <origin xyz="0.15 0.1 0"/>
         <material name="black"/>
       </visual>
     </link>

     <!-- Spherical Castor -->
     <link name="castor_wheel">
       <visual>
         <geometry>
           <sphere radius="0.05"/>
         </geometry>
         <origin xyz="0 0 -0.05"/>
         <material name="grey">
           <color rgba="0.5 0.5 0.5 1"/>
         </material>
       </visual>
     </link>

     <!-- Joints -->
     <joint name="left_wheel_joint" type="continuous">
       <parent link="base_link"/>
       <child link="left_wheel"/>
       <origin xyz="-0.15 0.1 0"/>
       <axis xyz="0 1 0"/>
     </joint>

     <joint name="right_wheel_joint" type="continuous">
       <parent link="base_link"/>
       <child link="right_wheel"/>
       <origin xyz="0.15 0.1 0"/>
       <axis xyz="0 1 0"/>
     </joint>

     <joint name="castor_joint" type="fixed">
       <parent link="base_link"/>
       <child link="castor_wheel"/>
       <origin xyz="0 0 -0.05"/>
     </joint>

   </robot>
   ```

3. **Launch the Robot in Gazebo:**

   Create a launch file to load the robot in Gazebo.

   ```bash
   touch launch/spawn_robot_launch.py
   ```

   Add the following content to the launch file:

   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='gazebo_ros',
               executable='spawn_entity.py',
               arguments=['-entity', 'simple_robot', '-file', 'urdf/simple_robot.urdf'],
               output='screen'
           ),
       ])
   ```

4. **Run the Launch File:**

   Execute the launch file to spawn your robot in Gazebo.

   ```bash
   ros2 launch my_robot spawn_robot_launch.py
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

2. **Modify Launch File to Include `ros2_control`:**

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

3. **Launch the Robot with Controllers:**

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
