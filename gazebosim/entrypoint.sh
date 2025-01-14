#!/bin/bash
set -e

# this sets up the display settings for use with Docker and Xserver in a windows environment
# Set DISPLAY environment variable if not set
if [ -z "$DISPLAY" ]; then
  export DISPLAY=host.docker.internal:0.0
fi

# Source ROS setup
source /opt/ros/humble/setup.bash

exec "$@"
