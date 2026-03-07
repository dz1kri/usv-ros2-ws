#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import requests
import json
import threading
from gz.transport13 import Node as GzNode
from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_v_pb2 import Pose_V

# Origin GPS (sesuai spherical_coordinates di waves.sdf)
ORIGIN_LAT = -5.85006661
ORIGIN_LNG = 106.80044651

def latlon_to_xy(lat, lon):
    """Konversi lat/lng ke meter relatif dari origin"""
    R = 6371000  # radius bumi meter
    dlat = math.radians(lat - ORIGIN_LAT)
    dlng = math.radians(lon - ORIGIN_LNG)
    x = dlng * R * math.cos(math.radians(ORIGIN_LAT))
    y = dlat * R
    return x, y

class GzWaypointNavigator(Node):
    def __init__(self):
        super().__init__('gz_waypoint_navigator')
        
        # Gazebo
        self.gz_node = GzNode()
        self.left_pub = self.gz_node.advertise(
            "/model/wam-v/joint/left_propeller_joint/cmd_thrust", Double)
        self.right_pub = self.gz_node.advertise(
            "/model/wam-v/joint/right_propeller_joint/cmd_thrust", Double)
        
        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.waypoints = []
        self.current_wp_index = 0
        self.navigating = False
        self.mission_id = None
        
        # SeaPortal API
        self.api_base_url = 'https://api.seano.cloud'
        self.vehicle_id = 20  # USV Gazebo Simulator
        self.auth_token = None
        
        # Subscribe pose Gazebo
        self.gz_node.subscribe(Pose_V, "/world/waves/dynamic_pose/info", self.on_pose)
        
        # Timers
        self.create_timer(0.1, self.navigate)
        self.create_timer(5.0, self.check_mission)
        
        # Login
        threading.Thread(target=self.login, daemon=True).start()
        self.get_logger().info('✅ GZ Waypoint Navigator ready!')

    def login(self):
        try:
            r = requests.post(f'{self.api_base_url}/auth/login',
                json={'email': 'seanouser@gmail.com', 'password': 'Seano2025*'})
            if r.status_code == 200:
                self.auth_token = r.json().get('access_token')
                self.get_logger().info('✅ Login SeaPortal berhasil!')
        except Exception as e:
            self.get_logger().error(f'Login error: {e}')

    def check_mission(self):
        if not self.auth_token or self.navigating:
            return
        try:
            headers = {'Authorization': f'Bearer {self.auth_token}'}
            r = requests.get(f'{self.api_base_url}/missions/', headers=headers)
            if r.status_code == 200:
                missions = r.json()
                for mission in missions:
                    if (mission.get('vehicle_id') == self.vehicle_id and
                            mission.get('status') == 'Ongoing' and
                            mission.get('id') != self.mission_id):
                        self.get_logger().info(f'🗺️ New mission: {mission["name"]}')
                        self.mission_id = mission['id']
                        waypoints = mission.get('waypoints', [])
                        xy_waypoints = []
                        for wp in waypoints:
                            x, y = latlon_to_xy(wp['lat'], wp['lng'])
                            xy_waypoints.append((x, y))
                            self.get_logger().info(f'  WP: lat={wp["lat"]:.6f} lng={wp["lng"]:.6f} -> x={x:.1f} y={y:.1f}')
                        self.set_waypoints(xy_waypoints)
        except Exception as e:
            self.get_logger().error(f'Mission check error: {e}')

    def on_pose(self, msg):
        for pose in msg.pose:
            if pose.name == "wam-v":
                self.current_x = pose.position.x
                self.current_y = pose.position.y
                q = pose.orientation
                self.current_yaw = math.atan2(
                    2*(q.w*q.z + q.x*q.y),
                    1 - 2*(q.y*q.y + q.z*q.z))

    def send_thrust(self, left, right):
        ml, mr = Double(), Double()
        ml.data, mr.data = left, right
        self.left_pub.publish(ml)
        self.right_pub.publish(mr)

    def set_waypoints(self, waypoints):
        self.waypoints = waypoints
        self.current_wp_index = 0
        self.navigating = True
        self.get_logger().info(f'📍 Navigating {len(waypoints)} waypoints')

    def navigate(self):
        if not self.navigating or not self.waypoints:
            return
        if self.current_wp_index >= len(self.waypoints):
            self.send_thrust(0, 0)
            self.navigating = False
            self.get_logger().info('✅ Mission complete!')
            self.update_mission_status('Completed')
            return
        wp = self.waypoints[self.current_wp_index]
        dx = wp[0] - self.current_x
        dy = wp[1] - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        if distance < 3.0:
            self.get_logger().info(f'✅ Waypoint {self.current_wp_index+1}/{len(self.waypoints)} reached!')
            self.current_wp_index += 1
            return
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.current_yaw
        while yaw_error > math.pi: yaw_error -= 2*math.pi
        while yaw_error < -math.pi: yaw_error += 2*math.pi
        base_thrust = -200.0
        correction = yaw_error * 80.0
        correction = max(-150, min(150, correction))
        self.send_thrust(base_thrust + correction, base_thrust - correction)

    def update_mission_status(self, status):
        if not self.auth_token or not self.mission_id:
            return
        try:
            headers = {'Authorization': f'Bearer {self.auth_token}'}
            requests.patch(f'{self.api_base_url}/missions/{self.mission_id}',
                json={'status': status}, headers=headers)
            self.get_logger().info(f'📡 Mission status: {status}')
        except Exception as e:
            self.get_logger().error(f'Update status error: {e}')

def main():
    rclpy.init()
    node = GzWaypointNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
