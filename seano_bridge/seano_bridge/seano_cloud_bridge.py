#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import requests
import json
import subprocess
import math
import ssl
from datetime import datetime
import paho.mqtt.client as mqtt

class SeanoCloudBridge(Node):
    def __init__(self):
        super().__init__('seano_cloud_bridge')
        
        # Configuration - FIXED VEHICLE ID
        self.api_base_url = 'https://api.seano.cloud'
        self.vehicle_id = 20  # ALWAYS USE VEHICLE 12
        self.vehicle_name = 'USV Gazebo Simulator'
        
        # MQTT Config
        self.mqtt_broker = 'd7b48187457f4e799b84ef68d8cf8783.s1.eu.hivemq.cloud'
        self.mqtt_port = 8884
        self.mqtt_username = 'seanoraspi'
        self.mqtt_password = 'Seanoraspi24*'
        
        # Auth
        self.auth_token = None
        
        # State
        self.current_pose = None
        self.current_mission = None
        self.current_waypoint_index = 0
        self.mission_active = False
        
        # Stats
        self.telemetry_success_count = 0
        self.telemetry_fail_count = 0
        
        # ROS Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/wamv/pose', self.pose_callback, 10
        )
        
        # Setup MQTT
        self.setup_mqtt()
        
        # Timers - Fast telemetry for stable connection
        self.telemetry_timer = self.create_timer(0.5, self.send_telemetry_http)
        self.status_timer = self.create_timer(2.0, self.update_vehicle_status)
        self.mission_timer = self.create_timer(3.0, self.check_mission)
        self.navigation_timer = self.create_timer(0.1, self.navigate)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('🌊 SEANO Cloud Bridge Started!')
        self.get_logger().info(f'🌐 API: {self.api_base_url}')
        self.get_logger().info(f'🚢 Vehicle ID: {self.vehicle_id} (FIXED)')
        self.get_logger().info('=' * 70)
        
        # Initialize
        self.login()
        
        # Verify vehicle exists
        if self.vehicle_id:
            self.get_logger().info(f'✅ Using Fixed Vehicle ID: {self.vehicle_id}')
    
    def setup_mqtt(self):
        """Setup MQTT client"""
        self.mqtt_client = mqtt.Client(
            client_id=f"usv_gazebo_v{self.vehicle_id}",
            clean_session=True
        )
        self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.mqtt_client.tls_set(cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLSv1_2)
        self.mqtt_client.tls_insecure_set(False)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=120)
        
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=120)
            self.mqtt_client.loop_start()
            self.get_logger().info('✅ MQTT setup complete')
        except Exception as e:
            self.get_logger().error(f'❌ MQTT failed: {e}')
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('✅ MQTT connected')
            if self.vehicle_id:
                client.subscribe(f'vehicle/{self.vehicle_id}/mission/#')
        else:
            self.get_logger().error(f'❌ MQTT failed: {rc}')
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn('⚠️ MQTT disconnected')
    
    def login(self):
        """Login to SEANO API"""
        try:
            login_data = {
                'email': 'seanouser@gmail.com',
                'password': 'Seano2025*'
            }
            response = requests.post(
                f'{self.api_base_url}/auth/login',
                json=login_data, timeout=10
            )
            if response.status_code == 200:
                data = response.json()
                self.auth_token = data.get('token') or data.get('access_token')
                self.get_logger().info('✅ Login successful')
                return True
            else:
                self.get_logger().error(f'❌ Login failed: {response.status_code}')
                return False
        except Exception as e:
            self.get_logger().error(f'❌ Login error: {e}')
            return False
    
    def get_headers(self):
        """Get HTTP headers with auth"""
        headers = {'Content-Type': 'application/json'}
        if self.auth_token:
            headers['Authorization'] = f'Bearer {self.auth_token}'
        return headers
    
    def pose_callback(self, msg):
        """Handle pose updates"""
        self.current_pose = msg
    
    def send_telemetry_http(self):
        """Send telemetry - optimized for stability"""
        if self.current_pose is None or self.vehicle_id is None:
            return
        
        try:
            x = self.current_pose.pose.position.x
            y = self.current_pose.pose.position.y
            z = self.current_pose.pose.position.z
            
            # Home location - Laut Jawa
            HOME_LAT = -5.8500
            HOME_LON = 106.8000
            
            lat_offset = y / 111320.0
            lon_offset = x / (111320.0 * math.cos(math.radians(HOME_LAT)))
            latitude = HOME_LAT + lat_offset
            longitude = HOME_LON + lon_offset
            
            telemetry = {
                'vehicle_id': self.vehicle_id,
                'timestamp': datetime.utcnow().isoformat() + 'Z',
                'latitude': latitude,
                'longitude': longitude,
                'altitude': z,
                'heading': 0.0,
                'speed': 0.0,
                'battery_percentage': 100,
                'status': 'active'
            }
            
            try:
                response = requests.post(
                    f'{self.api_base_url}/vehicle-logs/',
                    json=telemetry,
                    headers=self.get_headers(),
                    timeout=8
                )
                
                if response.status_code in [200, 201]:
                    self.telemetry_success_count += 1
                    self.get_logger().debug('✅ Telemetry sent')
                    
                    if self.telemetry_success_count % 60 == 0:
                        total = self.telemetry_success_count + self.telemetry_fail_count
                        rate = (self.telemetry_success_count / total) * 100 if total > 0 else 0
                        self.get_logger().info(f'📊 Success: {rate:.1f}% ({self.telemetry_success_count}/{total})')
                else:
                    self.telemetry_fail_count += 1
                    self.get_logger().warn(f'⚠️ Status {response.status_code}')
                    
            except requests.exceptions.Timeout:
                self.telemetry_fail_count += 1
                self.get_logger().warn('⚠️ Timeout')
            except Exception as e:
                self.telemetry_fail_count += 1
                self.get_logger().debug(f'Error: {type(e).__name__}')
                
        except Exception as e:
            self.get_logger().debug(f'Prep error: {e}')
    
    def update_vehicle_status(self):
        """Update vehicle status"""
        if self.vehicle_id is None:
            return
        
        try:
            status_data = {
                'status': 'active',
                'is_online': True,
                'last_seen': datetime.utcnow().isoformat() + 'Z'
            }
            response = requests.patch(
                f'{self.api_base_url}/vehicles/{self.vehicle_id}',
                json=status_data,
                headers=self.get_headers(),
                timeout=5
            )
            if response.status_code in [200, 204]:
                self.get_logger().debug('✅ Status updated')
        except:
            pass
    
    def check_mission(self):
        """Check for active missions"""
        if self.vehicle_id is None:
            return
        
        try:
            response = requests.get(
                f'{self.api_base_url}/missions/ongoing',
                headers=self.get_headers(),
                timeout=10
            )
            
            if response.status_code == 200:
                missions = response.json()
                for mission in missions:
                    if mission.get('vehicle_id') == self.vehicle_id:
                        if mission.get('id') != (self.current_mission.get('id') if self.current_mission else None):
                            self.current_mission = mission
                            self.current_waypoint_index = 0
                            self.mission_active = True
                            waypoints = mission.get('waypoints', [])
                            self.get_logger().info(f'🗺️ New mission: {mission.get("name")}')
                            self.get_logger().info(f'📍 Waypoints: {len(waypoints)}')
                        break
        except:
            pass
    
    def navigate(self):
        """Execute waypoint navigation"""
        if not self.mission_active or not self.current_mission or self.current_pose is None:
            return
        
        waypoints = self.current_mission.get('waypoints', [])
        if self.current_waypoint_index >= len(waypoints):
            self.get_logger().info('✅ Mission completed!')
            self.mission_active = False
            self.send_thrust(0.0, 0.0)
            return
        
        wp = waypoints[self.current_waypoint_index]
        target_lat = wp.get('latitude', 0)
        target_lon = wp.get('longitude', 0)
        target_x = target_lon * 111320.0 * math.cos(math.radians(target_lat))
        target_y = target_lat * 111320.0
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 2.0:
            self.get_logger().info(f'✅ Waypoint {self.current_waypoint_index + 1}/{len(waypoints)} reached!')
            self.current_waypoint_index += 1
            return
        
        thrust = min(distance * 10.0, 50.0)
        self.send_thrust(thrust, thrust)
    
    def send_thrust(self, left, right):
        """Send thrust commands"""
        try:
            subprocess.Popen(
                ['gz', 'topic', '-t', '/model/wam-v/joint/left_propeller_joint/cmd_thrust',
                 '-m', 'gz.msgs.Double', '-p', f'data: {left}'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            subprocess.Popen(
                ['gz', 'topic', '-t', '/model/wam-v/joint/right_propeller_joint/cmd_thrust',
                 '-m', 'gz.msgs.Double', '-p', f'data: {right}'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
        except Exception as e:
            self.get_logger().error(f'Thrust: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SeanoCloudBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
