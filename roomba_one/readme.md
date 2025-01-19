# ROS 2 Humble Docker Container

This repository contains a Dockerfile for setting up a development environment based on ROS 2 Humble Hawksbill on Ubuntu 22.04, tailored for use with Gazebo and a variety of ROS 2 packages. The intention is to provide a convenient and portable environment for robotics development and simulation.

We will use this container to build and simulate our first robot, a basic roomba differential drive robot.

## Overview

The Dockerfile is designed to pull an image from the official ROS 2 source for a full desktop installation, including tools for simulation and robot control. This environment includes essential tools and packages to start developing and testing robotics applications, especially those requiring Gazebo simulation or ROS 2 control.

## Features

- **Base Image**: Utilizes the `osrf/ros:humble-desktop-full` image to provide a comprehensive ROS 2 Humble setup on Ubuntu 22.04.
- **Bash Environment**: Configured to use bash as the default shell.
- **Interactive Shell Support**: Sources the `setup.bash` of ROS 2 in `.bashrc` to ensure that all interactive shells have the ROS 2 environment readily available.
- **Essential Packages**: Installs several packages necessary for development:
  - `git` for version control.
  - `python3-colcon-common-extensions` for building and managing ROS 2 workspaces.
  - `python3-rosdep` for managing dependencies.
  - `ros-humble-gazebo-ros-pkgs` and `ros-humble-gazebo-ros2-control` for integrating ROS 2 with Gazebo.
  - Control packages like `ros-humble-twist-mux`, `ros-humble-teleop-twist-keyboard`, `ros-humble-joy`, `ros-humble-ros2-control`, `ros-humble-ros2-controllers` for robot teleoperation and control configuration.
  - `nano` as a simple text editor for file manipulation within the container.


### Keeping the Container Running

The command `CMD ["bash"]` ensures that starting the container opens a bash shell, keeping the container active for interactive use.

## Cleanup

The Dockerfile removes cached package lists to minimize the final image size, following best practices to save space and ensure a clean environment.

## Additional Notes

- The container is configured with `DEBIAN_FRONTEND=noninteractive` to suppress any unnecessary package configuration prompts during installation, making automated builds smoother.
- To persist any data or changes, you may want to mount a volume between your host machine and the Docker container.




## Usage

Once the container is running, you can start developing your ROS2 applications with GUI support. The container includes all the necessary tools and libraries for ROS2 development.

## Instructions for lab 1
1. **Build the container**
```bash
docker build -f Dockerfile_roomba -t ros2_roomba .
```

2. **running lab_one container**
```bash
docker build -f Dockerfile_roomba -t ros2_roomba .
docker run -it --rm --name ros2_roomba_container \ 
    -e DISPLAY=host.docker.internal:0.0 \
    -v lab_one:/home/ros2_ws ros2_roomba
```

3. **Next steps**
- Follow the instructions in the lab_one readme.md file

## Generic instructions for building and using this container

1. **Build the Docker image:**
    ```bash
    docker build -t ros2_humble_gui .
    ```

2. **Run the Docker container:**
    The following is a general example of how to run this Docker container
    ### Linux:
    ```bash
    docker run -it --rm --name ros2_humble_gui_container -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ros2_humble_gui
    ```
    ### windows:
    Note: ensure you have Xlaunch installed and running, otherwise gui won't work
    ```bash
    docker run -it --rm --name ros2_humble_gui_container -e DISPLAY=host.docker.internal:0.0 ros2_humble_gui -v my_volume:/path/in/container my_image:latest
    ```
    -e defines the display settings. We need this for X11 support
    -v specifies the shared volume we'll use

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your changes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgements

- [ROS2](https://index.ros.org/doc/ros2/)
- [Docker](https://www.docker.com/)

For any questions or issues, please open an issue on the repository.
