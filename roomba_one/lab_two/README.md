### Control the roomba with ros2_control and teleop_twist

**Objective**: In this lab exercise, you will learn how to build a simple robot model in Gazebo with two cylinder wheels and a spherical castor wheel. You will then set up the robot for differential drive control using `ros2_control` and control it using `teleop_twist_keyboard`.

#### Prerequisites

- ROS 2 Humble installed
- Gazebo simulator installed
- Understanding of URDF (Unified Robot Description Format) and basic ROS 2 concepts
- Lab one completed

#### Instructions

---
Before beginning the next sections, update your workspace with the config, description and launch files provided in this folder.

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


1. **Adding `ros2_control`:**

   Add the following files to your project `config` folder:
   - gaz_ros2_use_sim.yaml
   - gazebo_params.yaml
   - joystick.yaml
   - my_controllers.yaml
   - twist_mux.yaml

   gaz_ros2_use_sim.yaml and gazebo_params.yaml are simple configuration files that set sim time to true and set the update rate.
   joystick.yaml provides some basic configurations for joy_node and teleop_node, which will be used to control the robot
   my_controllers.yaml sets up the diff_cont, our differential driver controller, and joint_broad, our joint state broadcaster.
   twist_mux.yaml configures a multiplexer for various input controllers. For example, if we want to be able to control our robot using an autonomous navigation 
   algorithm, but want to retain the option of manual teleoperation, we can multiplex the inputs, and also set priorities for each input
   
   You can find these files in the lab_two `config` folder in this repository

   2. **Update the launch file to load and start the controller:**

   ```python
   import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node

    def generate_launch_description():
        package_name = 'roomba_one'  # Ensure this matches your package name
        rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')]
            ),
            launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
        )

        joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory(package_name), 'launch', 'joystick.launch.py')]
            ),
            launch_arguments={'use_sim_time': 'true'}.items()
        )

        twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
        twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
        )

        gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
            ),
            launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
        )

        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
            output='screen'
        )

        # Use ExecuteProcess for the ROS 2 CLI commands instead of as nodes
        load_diff_drive_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_cont'],
            output='screen'
        )

        load_joint_broad_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_broad'],
            output='screen'
        )

        # Applying TimerAction to space out the execution order
        diff_drive_spawner = TimerAction(
            period=5.0,
            actions=[load_diff_drive_controller]
        )

        joint_broad_spawner = TimerAction(
            period=6.0,
            actions=[load_joint_broad_controller]
        )

        return LaunchDescription([
            rsp,
            joystick,
            twist_mux,
            gazebo,
            spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner
        ])
   ```

4. **Launch the Robot with Controllers:**

   Run the launch file to spawn the robot along with the controllers.

   ```bash
   ros2 launch roomba_one launch_sim.launch.py
   ```

### Part 3: Controlling the Robot with `teleop_twist_keyboard`

1. **Install `teleop_twist_keyboard`:**

   If you haven't already, install the package using:

   ```bash
   sudo apt-get install ros-<ros2-distro>-teleop-twist-keyboard
   ```

2. **Control the Robot via Keyboard:**

   Open a new terminal and run the keyboard teleoperation node:
   ```powershell
   docker exec -it ros2_roomba_container /bin/bash
   ```
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

   Use the keyboard controls to move the robot around in Gazebo. The node translates your key presses into velocity messages sent to the `/cmd_vel` topic, which 
   will move your robot.

### Part 4: Create a house and save to a .world file
As discussed in the lectures, we can create a simple house and insert models to create a more realistic simulation environment. 
- Create a simple 3-room house using the Gazebo building editor (refer to lecture notes)
- Insert furniture into the house, again as described in class
- Save your .world file in the following folder: /home/ros2_ws/src/worlds/
- restart Gazebo and spawn your new world with the world argument

```bash
ros2 launch roomba_one launch_sim.launch.py use_sim_time:=true world:=src/worlds/demo.world
```

#### Conclusion
At this stage, you should be able to teleoperate your Roomba around the virtual environment you created. Take time to review the config yaml files and updated launch file, to get a better feel for your code.
Review the design and implementation of your robot, with the goal of creating realistic movement. For example, it is likely your robot will pitch back when you first try to operate it. Think of ways to counter this in the robot design
Also, ensure your house is at a realistic scale and is somewhat realistic. Avoid walls protruding through other walls, or other undesirable effects.
Next week, we will add some sensors to our roomba.
