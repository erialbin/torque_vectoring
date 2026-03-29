#!/usr/bin/env python3
"""
Launch file for dv_template_package.

This launch file starts the time_publisher and time_subscriber nodes
with parameters loaded from the config file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for dv_template_package."""

    # Get the package share directory
    pkg_share = get_package_share_directory("dv_template_package")

    # Path to the parameters file
    params_file = os.path.join(pkg_share, "config", "parameters.yaml")

    return LaunchDescription(
        [
            # Time Publisher Node (C++)
            Node(
                package="dv_template_package",
                executable="example_time_publisher",
                name="time_publisher",
                output="screen",
                parameters=[params_file],
            ),
            # Time Subscriber Node (Python)
            Node(
                package="dv_template_package",
                executable="example_time_subscriber.py",
                name="time_subscriber",
                output="screen",
            ),
        ]
    )
