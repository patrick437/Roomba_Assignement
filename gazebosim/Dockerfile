# Use the ROS 2 Humble desktop full image as base
FROM osrf/ros:humble-desktop-full

# Set the non-interactive frontend for Debian to avoid prompts during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Update the package list and install necessary packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        lsb-release \  
        gnupg \        
        curl && \      
    # Add the Gazebo package repository key
    curl -sSL http://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    # Add the Gazebo package repository to the sources list
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    # Update the package list again and install additional packages
    # mesa-utils is required for glxinfo (graphics card support)
    apt-get update && \
    apt-get install -y --no-install-recommends \
        gz-ionic \  
        git \       
        python3-colcon-common-extensions \  
        python3-rosdep \  
        mesa-utils \ 
        libgl1-mesa-dri \  
        vainfo \  
        libva-dev && \  
    # Upgrade all installed packages to the latest version
    apt-get upgrade -y && \
    # Clean up the package lists to reduce the image size
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Use bash as the default shell
SHELL ["/bin/bash", "-c"]

# Create a temporary runtime directory for the root user and set permissions
RUN mkdir -p /tmp/runtime-root && chmod 0700 /tmp/runtime-root

# Copy the entrypoint script into the container
COPY entrypoint.sh /entrypoint.sh
# Make the entrypoint script executable
RUN chmod +x /entrypoint.sh

# Set the entrypoint to the entrypoint script
ENTRYPOINT ["/entrypoint.sh"]
# Default command to run if no other command is specified
CMD ["bash"]
