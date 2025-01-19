# ROS 2 Docker Environment with Gazebo

This repository contains a Docker setup for running ROS 2 Humble with Gazebo and necessary development tools. It allows you to easily get started with simulation environments without installing ROS 2 directly on your host machine.

## Prerequisites

Before you start, make sure you have the following installed on your system:

- **Docker**: [Install Docker](https://docs.docker.com/get-docker/) for your operating system.
- **Docker Compose**: [Install Docker Compose](https://docs.docker.com/compose/install/) if you plan on using Docker Compose for managing multi-container applications.
- **X Server**: Required for GUI applications. For Windows, you can use an X server like Xming or VcXsrv. For macOS, XQuartz is recommended.

## Setup

Follow these steps to set up and run the ROS 2 Docker environment with Gazebo.

### Step 1: Build the Docker Image

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/ros2-gazebo-docker.git
   cd ros2-gazebo-docker
   ```

2. Build the Docker image:
   ```bash
   docker build -t ros2-gazebo:humble .
   ```

### Step 2: Run the Docker Container

1. Start the X server on your host machine to allow GUI applications to connect.

2. Run the Docker container with the appropriate display settings:
   ```bash
   docker run -it --privileged --rm --name ros2_gazebo \
     -e DISPLAY=host.docker.internal:0.0 \
     --network host \
     ros2-gazebo:humble
   ```

   **Note**: Adjust the `DISPLAY` environment variable as needed based on your system configuration.

### Additional Configuration

The `entrypoint.sh` script sources the ROS 2 setup and configures environment variables required for Gazebo. You can customize this script to suit your specific development needs.

### Using the Container

Once the container is running, you can:
- Launch Gazebo and other ROS 2 tools from within the container.
- Develop and test ROS 2 applications with direct access to Gazebo for simulations.

### Troubleshooting

- **Display Issues**: Ensure your X server is running and allows connections. You may need to configure it to accept network connections.
- **Networking**: The container uses host networking. You might need to adjust network settings based on your specific configuration and security requirements.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## Acknowledgments

- [Open Source Robotics Foundation (OSRF)](https://www.openrobotics.org/) for ROS and Gazebo.


