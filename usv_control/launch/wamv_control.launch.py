#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Bridge ROS2-Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            arguments=[
                '/wamv/thrusters/left/thrust@std_msgs/msg/Float64@gz.msgs.Double',
                '/wamv/thrusters/right/thrust@std_msgs/msg/Float64@gz.msgs.Double',
            ],
            output='screen'
        ),
        
        # Teleop Keyboard
        Node(
            package='usv_control',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen'
        ),
    ])
