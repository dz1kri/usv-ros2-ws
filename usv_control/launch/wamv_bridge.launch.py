from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Bridge node
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/wamv/left_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/wamv/right_thrust@std_msgs/msg/Float64@gz.msgs.Double',
        ],
        output='screen'
    )
    
    return LaunchDescription([bridge])
