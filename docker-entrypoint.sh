#!/bin/bash
set -e
source /root/.bashrc

ROS_DISTRO=jazzy

source "/opt/ros/$ROS_DISTRO/setup.bash"

if [ ! -e ".CONTAINER_INITIALIZED_PLACEHOLDER" ]; then
    echo "-- First container startup --"
    
    # Build the workspace using colcon
    colcon build --symlink-install
    
    # This placeholder file used in the github action to check when colcon build is done, do not remove
    touch ".CONTAINER_INITIALIZED_PLACEHOLDER"
    
    # Source the workspace in every new shell to avoid sourcing it manually
    echo "source /ws/install/setup.bash" >> /root/.bashrc
else
    echo "-- Not first container startup --"
    source "/ws/install/setup.bash"
fi

# Execute the container command
exec "$@"
