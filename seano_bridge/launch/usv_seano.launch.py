from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # SeaPortal Cloud Bridge
        Node(
            package='seano_bridge',
            executable='bridge',
            name='seano_cloud_bridge'
        ),
        # MAVROS <-> SeaPortal Bridge
        Node(
            package='usv_control',
            executable='mavros_seano_bridge',
            name='mavros_seano_bridge'
        ),
        # Mission Executor dari SeaPortal
        Node(
            package='usv_control',
            executable='seano_mission_executor',
            name='seano_mission_executor'
        ),
        # Dashboard Server
        Node(
            package='usv_dashboard',
            executable='dashboard',
            name='usv_dashboard'
        ),
    ])
