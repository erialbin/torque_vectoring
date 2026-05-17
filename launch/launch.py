#!/usr/bin/env python3
"""
Launch file for torque_vectoring_pkg.

This launch file starts the time_publisher and time_subscriber nodes
with parameters loaded from the config file.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for torque_vectoring_pkg."""

    # Get the package share directory
    # pkg_share = get_package_share_directory("torque_vectoring_pkg")

    # Path to the parameters file
    # params_file = os.path.join(pkg_share, "config", "parameters.yaml")

    return LaunchDescription(
        [
            # Time Publisher Node (C++)
            Node(
                package="torque_vectoring_pkg",
                executable="torque_vectoring_pkg",
                name="torque_vectoring_node",
                output="screen",
                remappings=[
                    ("steering_cmd", "navigation/dv_control_target"),
                    ("imu", "dlio/odom"),
                ],
            ),
        ]
    )
