#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GzDynamicPoseBridge(Node):
    def __init__(self):
        super().__init__('gz_dynamic_pose_bridge')
        
        self.pose_pub = self.create_publisher(PoseStamped, '/wamv/pose', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)
        
        self.get_logger().info('✅ Gazebo Dynamic Pose Bridge started')
    
    def get_model_pose(self):
        """Get model pose - HARDCODED FOR TESTING"""
        import time
        import math
        
        t = time.time()
        
        # Simulate boat moving in circle (50m radius)
        radius = 50.0
        x = radius * math.cos(t * 0.1)
        y = radius * math.sin(t * 0.1)
        
        return {'x': x, 'y': y, 'z': 0.2}
    
    def publish_pose(self):
        """Publish pose to ROS"""
        pose_data = self.get_model_pose()
        
        if pose_data:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            
            msg.pose.position.x = pose_data['x']
            msg.pose.position.y = pose_data['y']
            msg.pose.position.z = pose_data['z']
            
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            
            self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GzDynamicPoseBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
