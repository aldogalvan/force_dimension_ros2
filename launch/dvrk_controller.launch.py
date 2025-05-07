#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='force_dimension',
            executable='node',
            namespace='robot1',
        ),
        Node(
            package='force_dimension',
            executable='node',
            namespace='robot2',
        ),
        Node(
            package="force_dimension",
            executable="dvrk_controller"
        )
    ])