#!/usr/bin/env python3
"""
MAVROS → SeaPortal Bridge
Mengambil data telemetry dari MAVROS dan mengirimnya ke SeaPortal API
Mendukung hardware nyata (CUAV X7+) dan simulasi SITL
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
import requests
import json
import math
import ssl
from datetime import datetime
import paho.mqtt.client as mqtt


class MavrosSeanooBridge(Node):
    def __init__(self):
        super().__init__('mavros_seano_bridge')

        # === KONFIGURASI ===
        self.api_base_url = 'https://api.seano.cloud'
        self.vehicle_id = 1        # USV SEANO USV-001
        self.vehicle_code = 'USV-001'

        # MQTT
        self.mqtt_broker = 'd7b48187457f4e799b84ef68d8cf8783.s1.eu.hivemq.cloud'
        self.mqtt_port = 8884
        self.mqtt_username = 'seanoraspi'
        self.mqtt_password = 'Seanoraspi24*'

        # Auth
        self.auth_token = None

        # Telemetry state
        self.current_gps = None
        self.current_state = State()
        self.current_battery = None
        self.current_velocity = None
        self.heading = 0.0

        # Stats
        self.success_count = 0
        self.fail_count = 0

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # === SUBSCRIBERS ===
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos)

        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, qos)

        self.battery_sub = self.create_subscription(
            BatteryState, '/mavros/battery', self.battery_callback, qos)

        self.vel_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_body',
            self.velocity_callback, qos)

        # === TIMERS ===
        self.telemetry_timer = self.create_timer(1.0, self.send_telemetry)
        self.status_timer = self.create_timer(5.0, self.log_status)

        # === INIT ===
        self.get_logger().info('=' * 60)
        self.get_logger().info('🌊 MAVROS → SeaPortal Bridge Started')
        self.get_logger().info(f'🚢 Vehicle: {self.vehicle_code} (ID: {self.vehicle_id})')
        self.get_logger().info('=' * 60)

        self.setup_mqtt()
        self.login()

    # =========================================================
    # CALLBACKS
    # =========================================================
    def state_callback(self, msg):
        self.current_state = msg

    def gps_callback(self, msg):
        self.current_gps = msg

    def battery_callback(self, msg):
        self.current_battery = msg

    def velocity_callback(self, msg):
        self.current_velocity = msg
        # Hitung speed dari velocity
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.speed = math.sqrt(vx**2 + vy**2)
        # Hitung heading dari velocity
        if abs(vx) > 0.1 or abs(vy) > 0.1:
            self.heading = math.degrees(math.atan2(vy, vx)) % 360

    # =========================================================
    # AUTH
    # =========================================================
    def login(self):
        try:
            response = requests.post(
                f'{self.api_base_url}/auth/login',
                json={'email': 'seanouser@gmail.com', 'password': 'Seano2025*'},
                timeout=30
            )
            if response.status_code == 200:
                data = response.json()
                self.auth_token = data.get('access_token') or data.get('token')
                self.get_logger().info('✅ Login SeaPortal berhasil')
                return True
            self.get_logger().error(f'❌ Login gagal: {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'❌ Login error: {e}')
        return False

    def get_headers(self):
        headers = {'Content-Type': 'application/json'}
        if self.auth_token:
            headers['Authorization'] = f'Bearer {self.auth_token}'
        return headers

    # =========================================================
    # MQTT
    # =========================================================
    def setup_mqtt(self):
        try:
            self.mqtt_client = mqtt.Client(
                client_id=f'mavros_bridge_{self.vehicle_id}',
                clean_session=True
            )
            self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
            self.mqtt_client.tls_set(cert_reqs=ssl.CERT_REQUIRED,
                                      tls_version=ssl.PROTOCOL_TLSv1_2)
            self.mqtt_client.tls_insecure_set(False)
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=120)
            self.mqtt_client.loop_start()
            self.get_logger().info('✅ MQTT setup selesai')
        except Exception as e:
            self.get_logger().error(f'❌ MQTT error: {e}')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('✅ MQTT terhubung')
            client.subscribe(f'vehicle/{self.vehicle_id}/mission/#')
            client.subscribe(f'vehicle/{self.vehicle_id}/command/#')
        else:
            self.get_logger().error(f'❌ MQTT gagal: rc={rc}')

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn('⚠️ MQTT terputus, mencoba reconnect...')

    def on_mqtt_message(self, client, userdata, msg):
        """Handle perintah dari SeaPortal via MQTT"""
        try:
            topic = msg.topic
            payload = json.loads(msg.payload.decode())
            self.get_logger().info(f'📨 MQTT [{topic}]: {payload}')
            # TODO: Handle mission commands dari SeaPortal
        except Exception as e:
            self.get_logger().error(f'MQTT message error: {e}')

    # =========================================================
    # TELEMETRY
    # =========================================================
    def send_telemetry(self):
        """Kirim telemetry ke SeaPortal"""
        if self.current_gps is None:
            self.get_logger().debug('⏳ Menunggu GPS...')
            return

        # Cek kualitas GPS
        if self.current_gps.status.status < 0:
            self.get_logger().warn('⚠️ GPS fix belum ada')
            return

        try:
            # Battery
            battery_pct = 100.0
            if self.current_battery:
                battery_pct = self.current_battery.percentage * 100
                if battery_pct < 0:
                    battery_pct = 100.0

            # Speed
            speed = getattr(self, 'speed', 0.0)

            # Mode & status
            mode = self.current_state.mode if self.current_state else 'UNKNOWN'
            armed = self.current_state.armed if self.current_state else False

            telemetry = {
                'vehicle_id': self.vehicle_id,
                'latitude': self.current_gps.latitude,
                'longitude': self.current_gps.longitude,
                'altitude': self.current_gps.altitude,
                'heading': self.heading,
                'speed': round(speed, 2),
                'battery_percentage': round(battery_pct, 1),
                'status': 'active' if armed else 'standby',
                'mode': mode,
                'armed': armed,
                'gps_ok': self.current_gps.status.status >= 0,
            }
            # Kirim via HTTP
            response = requests.post(
                f'{self.api_base_url}/vehicle-logs/',
                json=telemetry,
                headers=self.get_headers(),
                timeout=8
            )

            if response.status_code in [200, 201]:
                self.success_count += 1
                self.get_logger().debug(
                    f'✅ Telemetry OK | '
                    f'GPS: {self.current_gps.latitude:.5f},{self.current_gps.longitude:.5f} | '
                    f'Mode: {mode} | Batt: {battery_pct:.0f}%'
                )

                # Kirim juga via MQTT
                self.mqtt_client.publish(
                    f'vehicle/{self.vehicle_id}/telemetry',
                    json.dumps(telemetry)
                )

            elif response.status_code == 401:
                self.get_logger().warn('⚠️ Token expired, login ulang...')
                self.login()
            else:
                self.fail_count += 1
                self.get_logger().warn(f'⚠️ Telemetry gagal: {response.status_code}')

        except requests.exceptions.Timeout:
            self.fail_count += 1
            self.get_logger().warn('⚠️ Timeout mengirim telemetry')
        except Exception as e:
            self.fail_count += 1
            self.get_logger().debug(f'Telemetry error: {e}')

    def log_status(self):
        """Log status secara berkala"""
        connected = self.current_state.connected if self.current_state else False
        mode = self.current_state.mode if self.current_state else 'N/A'
        armed = self.current_state.armed if self.current_state else False
        gps = 'OK' if self.current_gps else 'N/A'
        total = self.success_count + self.fail_count
        rate = (self.success_count / total * 100) if total > 0 else 0

        self.get_logger().info(
            f'📊 FCU: {"✅" if connected else "❌"} | '
            f'Mode: {mode} | Armed: {armed} | GPS: {gps} | '
            f'Telemetry: {rate:.0f}% ({self.success_count}/{total})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MavrosSeanooBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Bridge shutdown')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
