#!/usr/bin/env python3
"""
USV Full System Launch File
Menjalankan: MAVROS + SeaPortal Bridge + Mission Executor
Gunakan setelah SITL sudah jalan
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Argumen fcu_url
    # SITL  : udp://127.0.0.1:14550@14555
    # Hardware (USB) : /dev/ttyUSB0:57600
    # Hardware (UART): /dev/ttyACM0:115200
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://127.0.0.1:14550@14555',
        description='FCU URL'
    )

    fcu_url = LaunchConfiguration('fcu_url')

    # 1. MAVROS Node
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

    # 2. SeaPortal Bridge (delay 5 detik, tunggu MAVROS ready)
    bridge_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='usv_control',
                executable='mavros_seano_bridge',
                name='mavros_seano_bridge',
                output='screen'
            )
        ]
    )

    # 3. Mission Executor (delay 7 detik, tunggu bridge ready)
    mission_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='usv_control',
                executable='seano_mission_executor',
                name='seano_mission_executor',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        fcu_url_arg,
        mavros_node,
        bridge_node,
        mission_node,
    ])
