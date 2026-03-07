#!/usr/bin/env python3
"""
SeaPortal Mission Executor
Mengambil misi dari SeaPortal API dan mengeksekusinya via MAVROS AUTO mode
Alur: SeaPortal Mission Planner → API → Node ini → MAVROS → CUAV X7+
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from mavros_msgs.msg import State, Waypoint
from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear
from sensor_msgs.msg import NavSatFix
import requests
import math
import time
from datetime import datetime


class SeanoMissionExecutor(Node):
    def __init__(self):
        super().__init__('seano_mission_executor')

        # === KONFIGURASI ===
        self.api_base_url = 'https://api.seano.cloud'
        self.vehicle_id = 1  # USV SEANO USV-001
        self.auth_token = None

        # State
        self.current_state = State()
        self.current_gps = None
        self.active_mission_id = None
        self.mission_running = False

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

        # === SERVICE CLIENTS ===
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.wp_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.wp_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')

        # Tunggu services
        self.get_logger().info('⏳ Menunggu MAVROS services...')
        self.arming_client.wait_for_service(timeout_sec=15.0)
        self.set_mode_client.wait_for_service(timeout_sec=15.0)
        self.wp_push_client.wait_for_service(timeout_sec=15.0)
        self.wp_clear_client.wait_for_service(timeout_sec=15.0)
        self.get_logger().info('✅ MAVROS services ready!')

        # === TIMERS ===
        # Cek misi baru dari SeaPortal setiap 5 detik
        self.mission_check_timer = self.create_timer(5.0, self.check_and_execute_mission)
        # Update progress misi ke SeaPortal setiap 3 detik
        self.progress_timer = self.create_timer(3.0, self.update_mission_progress)
        # Log status setiap 10 detik
        self.status_timer = self.create_timer(10.0, self.log_status)

        # Login
        self.login()

        self.get_logger().info('=' * 60)
        self.get_logger().info('🌊 SeaPortal Mission Executor Started')
        self.get_logger().info(f'🚢 Vehicle ID: {self.vehicle_id}')
        self.get_logger().info('🔄 Menunggu misi dari SeaPortal...')
        self.get_logger().info('=' * 60)

    # =========================================================
    # CALLBACKS
    # =========================================================
    def state_callback(self, msg):
        self.current_state = msg

    def gps_callback(self, msg):
        self.current_gps = msg

    # =========================================================
    # AUTH
    # =========================================================
    def login(self):
        try:
            response = requests.post(
                f'{self.api_base_url}/auth/login',
                json={'email': 'seanouser@gmail.com', 'password': 'Seano2025*'},
                timeout=10
            )
            if response.status_code == 200:
                self.auth_token = response.json().get('access_token')
                self.get_logger().info('✅ Login SeaPortal berhasil')
                return True
        except Exception as e:
            self.get_logger().error(f'❌ Login error: {e}')
        return False

    def get_headers(self):
        headers = {'Content-Type': 'application/json'}
        if self.auth_token:
            headers['Authorization'] = f'Bearer {self.auth_token}'
        return headers

    # =========================================================
    # CEK MISI DARI SEAPORTAL
    # =========================================================
    def check_and_execute_mission(self):
        """Cek misi ongoing dari SeaPortal dan eksekusi jika ada"""
        if not self.current_state.connected:
            return

        if self.mission_running:
            return

        try:
            response = requests.get(
                f'{self.api_base_url}/missions/ongoing',
                headers=self.get_headers(),
                timeout=8
            )

            if response.status_code == 401:
                self.login()
                return

            if response.status_code != 200:
                return

            missions = response.json()

            # Cari misi untuk vehicle ini
            for mission in missions:
                if mission.get('vehicle_id') == self.vehicle_id:
                    mission_id = mission.get('id')

                    # Misi baru yang belum dieksekusi
                    if mission_id != self.active_mission_id:
                        self.get_logger().info(
                            f'🗺️ Misi baru ditemukan: [{mission_id}] {mission.get("name")}')
                        self.execute_mission(mission)
                    break

        except requests.exceptions.Timeout:
            self.get_logger().debug('Timeout cek misi')
        except Exception as e:
            self.get_logger().debug(f'Cek misi error: {e}')

    # =========================================================
    # EKSEKUSI MISI
    # =========================================================
    def execute_mission(self, mission: dict):
        """Eksekusi misi dari SeaPortal via MAVROS AUTO mode"""
        mission_id = mission.get('id')
        mission_name = mission.get('name')
        waypoints_raw = mission.get('waypoints', [])

        if not waypoints_raw:
            self.get_logger().warn(f'⚠️ Misi {mission_id} tidak punya waypoints!')
            return

        self.get_logger().info(f'🚀 Memulai eksekusi misi: {mission_name}')
        self.get_logger().info(f'📍 Jumlah waypoints: {len(waypoints_raw)}')

        # Convert format SeaPortal (lat/lng) ke format internal
        waypoints = [{'lat': wp['lat'], 'lon': wp['lng']} for wp in waypoints_raw]

        # 1. Upload waypoints ke ArduPilot
        if not self.upload_waypoints(waypoints):
            self.get_logger().error('❌ Gagal upload waypoints!')
            self.update_mission_status(mission_id, 'Failed')
            return

        # 2. Set mode AUTO
        if not self.set_mode('AUTO'):
            self.get_logger().error('❌ Gagal set mode AUTO!')
            return

        # 3. Arm vehicle
        time.sleep(1.0)
        if not self.current_state.armed:
            if not self.arm():
                self.get_logger().error('❌ Gagal arm vehicle!')
                return

        # 4. Update status misi ke "ongoing" di SeaPortal
        self.active_mission_id = mission_id
        self.mission_running = True
        self.update_mission_status(mission_id, 'Ongoing')

        self.get_logger().info(f'✅ Misi [{mission_id}] {mission_name} berjalan!')

    # =========================================================
    # UPLOAD WAYPOINTS KE ARDUPILOT
    # =========================================================
    def upload_waypoints(self, waypoints: list) -> bool:
        """Upload waypoints ke ArduPilot via MAVROS"""

        # Clear misi lama
        clear_req = WaypointClear.Request()
        future = self.wp_clear_client.call_async(clear_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        wp_list = []

        # Home waypoint (index 0) - gunakan posisi GPS saat ini
        home = Waypoint()
        home.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        home.command = 16
        home.is_current = False
        home.autocontinue = True
        if self.current_gps:
            home.x_lat = self.current_gps.latitude
            home.y_long = self.current_gps.longitude
        else:
            home.x_lat = waypoints[0]['lat']
            home.y_long = waypoints[0]['lon']
        home.z_alt = 0.0
        wp_list.append(home)

        # Waypoints dari SeaPortal
        for i, wp in enumerate(waypoints):
            waypoint = Waypoint()
            waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            waypoint.command = 16  # MAV_CMD_NAV_WAYPOINT
            waypoint.is_current = (i == 0)
            waypoint.autocontinue = True
            waypoint.param1 = 0.0    # Hold time
            waypoint.param2 = 3.0    # Acceptance radius (meter)
            waypoint.param3 = 0.0    # Pass radius
            waypoint.param4 = float('nan')  # Yaw
            waypoint.x_lat = wp['lat']
            waypoint.y_long = wp['lon']
            waypoint.z_alt = 0.0
            wp_list.append(waypoint)

        # Push ke ArduPilot
        push_req = WaypointPush.Request()
        push_req.start_index = 0
        push_req.waypoints = wp_list
        future = self.wp_push_client.call_async(push_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() and future.result().success:
            self.get_logger().info(
                f'✅ {future.result().wp_transfered} waypoints uploaded ke ArduPilot')
            return True

        self.get_logger().error('❌ Upload waypoints gagal!')
        return False

    # =========================================================
    # ARM / MODE
    # =========================================================
    def arm(self) -> bool:
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success:
            self.get_logger().info('✅ Vehicle armed!')
            return True
        self.get_logger().error('❌ Arming gagal!')
        return False

    def disarm(self) -> bool:
        req = CommandBool.Request()
        req.value = False
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() and future.result().success

    def set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f'✅ Mode {mode} aktif')
            return True
        self.get_logger().error(f'❌ Set mode {mode} gagal!')
        return False

    # =========================================================
    # UPDATE STATUS MISI KE SEAPORTAL
    # =========================================================
    def update_mission_status(self, mission_id: int, status: str):
        """Update status misi di SeaPortal"""
        try:
            data = {'status': status}
            if status == 'Ongoing':
                data['start_time'] = datetime.utcnow().isoformat() + 'Z'
            elif status in ['Completed', 'Failed']:
                data['end_time'] = datetime.utcnow().isoformat() + 'Z'

            response = requests.patch(
                f'{self.api_base_url}/missions/{mission_id}',
                json=data,
                headers=self.get_headers(),
                timeout=8
            )
            if response.status_code in [200, 204]:
                self.get_logger().info(f'✅ Status misi {mission_id} → {status}')
        except Exception as e:
            self.get_logger().debug(f'Update status error: {e}')

    def update_mission_progress(self):
        """Update progress misi berdasarkan waypoint yang sudah dicapai"""
        if not self.mission_running or not self.active_mission_id:
            return

        try:
            # Cek mode - kalau sudah tidak AUTO berarti misi selesai
            if self.current_state.mode not in ['AUTO', 'GUIDED']:
                if not self.current_state.armed:
                    self.get_logger().info('🏁 Misi selesai!')
                    self.update_mission_status(self.active_mission_id, 'Completed')
                    self.mission_running = False
                    self.active_mission_id = None

        except Exception as e:
            self.get_logger().debug(f'Progress update error: {e}')

    # =========================================================
    # EMERGENCY STOP
    # =========================================================
    def emergency_stop(self):
        """Stop darurat - set HOLD dan disarm"""
        self.get_logger().warn('🚨 EMERGENCY STOP!')
        self.set_mode('HOLD')
        time.sleep(0.5)
        self.disarm()
        if self.active_mission_id:
            self.update_mission_status(self.active_mission_id, 'Failed')
        self.mission_running = False
        self.active_mission_id = None

    def log_status(self):
        connected = self.current_state.connected
        mode = self.current_state.mode
        armed = self.current_state.armed
        gps = 'OK' if self.current_gps else 'N/A'
        mission = f'ID:{self.active_mission_id}' if self.active_mission_id else 'Menunggu'

        self.get_logger().info(
            f'📊 FCU: {"✅" if connected else "❌"} | '
            f'Mode: {mode} | Armed: {armed} | GPS: {gps} | '
            f'Misi: {mission}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SeanoMissionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutdown - Emergency stop...')
        node.emergency_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
