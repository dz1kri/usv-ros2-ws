#!/usr/bin/env python3
"""
Launch file USV Autonomous System
Menjalankan: MAVROS + Autonomous Control + SeaPortal Bridge
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Argumen
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://127.0.0.1:14550@14555',
        description='FCU URL: SITL=udp://127.0.0.1:14550@14555, Hardware=/dev/ttyUSB0:57600'
    )

    fcu_url = LaunchConfiguration('fcu_url')

    # MAVROS Node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace='/mavros',
        parameters=[{
            'fcu_url': fcu_url,
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',
        }],
        output='screen'
    )

    # Autonomous Control Node
    autonomous_node = Node(
        package='usv_control',
        executable='autonomous_control',
        name='usv_autonomous_control',
        output='screen'
    )

    # SeaPortal Bridge Node
    bridge_node = Node(
        package='usv_control',
        executable='mavros_seano_bridge',
        name='mavros_seano_bridge',
        output='screen'
    )

    return LaunchDescription([
        fcu_url_arg,
        mavros_node,
        autonomous_node,
        bridge_node,
    ])
