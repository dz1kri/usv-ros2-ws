#!/usr/bin/env python3
"""
USV Autonomous Control Node
Mendukung AUTO mode (upload misi) dan GUIDED mode (waypoint dinamis)
Terhubung ke ArduPilot via MAVROS
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import State, Waypoint, WaypointList
from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear, CommandLong
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import math
import time


class USVAutonomousControl(Node):
    def __init__(self):
        super().__init__('usv_autonomous_control')

        # QoS profile untuk MAVROS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # === STATE ===
        self.current_state = State()
        self.current_gps = None
        self.armed = False
        self.current_mode = ''

        # === SUBSCRIBERS ===
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos)

        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, qos)

        # === PUBLISHERS ===
        # Untuk GUIDED mode - kirim target posisi
        self.guided_pub = self.create_publisher(
            GeoPoseStamped, '/mavros/setpoint_position/global', 10)

        # === SERVICE CLIENTS ===
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.wp_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.wp_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')

        # Tunggu services ready
        self.get_logger().info('⏳ Menunggu MAVROS services...')
        self.arming_client.wait_for_service(timeout_sec=10.0)
        self.set_mode_client.wait_for_service(timeout_sec=10.0)
        self.wp_push_client.wait_for_service(timeout_sec=10.0)
        self.wp_clear_client.wait_for_service(timeout_sec=10.0)

        self.get_logger().info('✅ MAVROS services ready!')
        self.get_logger().info('🚢 USV Autonomous Control Node Started')

    # =========================================================
    # CALLBACKS
    # =========================================================
    def state_callback(self, msg):
        self.current_state = msg
        self.armed = msg.armed
        self.current_mode = msg.mode

    def gps_callback(self, msg):
        self.current_gps = msg

    # =========================================================
    # ARMING
    # =========================================================
    def arm(self):
        """Arm vehicle"""
        self.get_logger().info('🔑 Arming...')
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success:
            self.get_logger().info('✅ Armed!')
            return True
        self.get_logger().error('❌ Arming gagal!')
        return False

    def disarm(self):
        """Disarm vehicle"""
        self.get_logger().info('🔒 Disarming...')
        req = CommandBool.Request()
        req.value = False
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success:
            self.get_logger().info('✅ Disarmed!')
            return True
        return False

    # =========================================================
    # MODE SWITCHING
    # =========================================================
    def set_mode(self, mode: str):
        """Set flight mode: HOLD, AUTO, GUIDED, MANUAL"""
        self.get_logger().info(f'🔄 Set mode: {mode}')
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f'✅ Mode {mode} aktif!')
            return True
        self.get_logger().error(f'❌ Set mode {mode} gagal!')
        return False

    # =========================================================
    # AUTO MODE - Upload waypoints ke flight controller
    # =========================================================
    def upload_mission(self, waypoints: list):
        """
        Upload misi waypoints ke ArduPilot dalam AUTO mode
        waypoints: list of dict {'lat': float, 'lon': float, 'alt': float}
        
        Contoh:
        waypoints = [
            {'lat': -6.89, 'lon': 107.56, 'alt': 0.0},
            {'lat': -6.90, 'lon': 107.57, 'alt': 0.0},
        ]
        """
        self.get_logger().info(f'📤 Upload misi: {len(waypoints)} waypoints')

        # Clear misi lama
        clear_req = WaypointClear.Request()
        future = self.wp_clear_client.call_async(clear_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        # Buat waypoint list
        wp_list = []

        # Home waypoint (index 0) - wajib ada
        home = Waypoint()
        home.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        home.command = 16  # MAV_CMD_NAV_WAYPOINT
        home.is_current = False
        home.autocontinue = True
        home.x_lat = waypoints[0]['lat']
        home.y_long = waypoints[0]['lon']
        home.z_alt = 0.0
        wp_list.append(home)

        # Waypoints
        for i, wp in enumerate(waypoints):
            waypoint = Waypoint()
            waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            waypoint.command = 16  # MAV_CMD_NAV_WAYPOINT
            waypoint.is_current = (i == 0)
            waypoint.autocontinue = True
            waypoint.param1 = 0.0   # Hold time
            waypoint.param2 = 2.0   # Acceptance radius (meter)
            waypoint.param3 = 0.0   # Pass radius
            waypoint.param4 = float('nan')  # Yaw
            waypoint.x_lat = wp['lat']
            waypoint.y_long = wp['lon']
            waypoint.z_alt = wp.get('alt', 0.0)
            wp_list.append(waypoint)

        # Push ke ArduPilot
        push_req = WaypointPush.Request()
        push_req.start_index = 0
        push_req.waypoints = wp_list
        future = self.wp_push_client.call_async(push_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() and future.result().success:
            self.get_logger().info(f'✅ Misi uploaded! {future.result().wp_transfered} waypoints')
            return True
        self.get_logger().error('❌ Upload misi gagal!')
        return False

    def start_auto_mission(self, waypoints: list):
        """
        Jalankan misi AUTO: upload waypoints lalu set mode AUTO
        """
        self.get_logger().info('🗺️ === MEMULAI AUTO MISSION ===')

        if not self.current_state.connected:
            self.get_logger().error('❌ FCU tidak terhubung!')
            return False

        # Upload waypoints
        if not self.upload_mission(waypoints):
            return False

        # Set mode AUTO
        if not self.set_mode('AUTO'):
            return False

        # Arm
        if not self.armed:
            time.sleep(1.0)
            if not self.arm():
                return False

        self.get_logger().info('🚢 AUTO Mission berjalan!')
        return True

    # =========================================================
    # GUIDED MODE - Kirim waypoint satu per satu
    # =========================================================
    def go_to_waypoint(self, lat: float, lon: float, alt: float = 0.0):
        """
        Kirim USV ke koordinat tertentu dalam GUIDED mode
        """
        self.get_logger().info(f'📍 GUIDED → Lat: {lat:.6f}, Lon: {lon:.6f}')

        # Set GUIDED mode jika belum
        if self.current_mode != 'GUIDED':
            if not self.set_mode('GUIDED'):
                return False

        # Arm jika belum
        if not self.armed:
            time.sleep(1.0)
            if not self.arm():
                return False

        # Publish target position
        target = GeoPoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'
        target.pose.position.latitude = lat
        target.pose.position.longitude = lon
        target.pose.position.altitude = alt
        target.pose.orientation.w = 1.0

        # Publish beberapa kali agar diterima
        for _ in range(10):
            self.guided_pub.publish(target)
            time.sleep(0.05)

        self.get_logger().info(f'✅ Target dikirim!')
        return True

    def guided_mission(self, waypoints: list, acceptance_radius: float = 3.0):
        """
        Jalankan misi GUIDED: navigasi waypoint satu per satu dengan monitoring
        waypoints: list of dict {'lat': float, 'lon': float}
        acceptance_radius: jarak (meter) untuk dianggap sampai
        """
        self.get_logger().info(f'🧭 === MEMULAI GUIDED MISSION ({len(waypoints)} waypoints) ===')

        for i, wp in enumerate(waypoints):
            self.get_logger().info(f'➡️ Menuju waypoint {i+1}/{len(waypoints)}')
            self.go_to_waypoint(wp['lat'], wp['lon'], wp.get('alt', 0.0))

            # Tunggu sampai di waypoint
            timeout = 120  # detik
            start = time.time()
            while time.time() - start < timeout:
                rclpy.spin_once(self, timeout_sec=0.5)

                if self.current_gps:
                    dist = self.haversine(
                        self.current_gps.latitude, self.current_gps.longitude,
                        wp['lat'], wp['lon']
                    )
                    self.get_logger().info(
                        f'📏 Jarak ke WP{i+1}: {dist:.1f}m', throttle_duration_sec=3.0)

                    if dist < acceptance_radius:
                        self.get_logger().info(f'✅ Waypoint {i+1} tercapai!')
                        break
                else:
                    self.get_logger().warn('⚠️ GPS belum tersedia...')

            else:
                self.get_logger().warn(f'⏱️ Timeout waypoint {i+1}, lanjut ke berikutnya')

        self.get_logger().info('🏁 GUIDED Mission selesai!')
        self.set_mode('HOLD')

    # =========================================================
    # OVERRIDE - Paksa GUIDED dari AUTO
    # =========================================================
    def override_to_guided(self, lat: float, lon: float):
        """
        Override dari AUTO ke GUIDED dan arahkan ke koordinat baru
        Berguna untuk menghindari rintangan atau mengubah tujuan
        """
        self.get_logger().warn('⚡ OVERRIDE: AUTO → GUIDED')
        self.set_mode('GUIDED')
        time.sleep(0.5)
        self.go_to_waypoint(lat, lon)

    def return_to_auto(self):
        """Kembali ke AUTO mode setelah override"""
        self.get_logger().info('↩️ Kembali ke AUTO mode')
        self.set_mode('AUTO')

    # =========================================================
    # UTILITY
    # =========================================================
    def haversine(self, lat1, lon1, lat2, lon2) -> float:
        """Hitung jarak (meter) antara dua koordinat GPS"""
        R = 6371000  # radius bumi meter
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    def get_status(self):
        """Print status saat ini"""
        gps_info = 'N/A'
        if self.current_gps:
            gps_info = f'{self.current_gps.latitude:.6f}, {self.current_gps.longitude:.6f}'
        self.get_logger().info(
            f'📊 Status | Connected: {self.current_state.connected} | '
            f'Mode: {self.current_mode} | Armed: {self.armed} | GPS: {gps_info}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = USVAutonomousControl()

    # =========================================================
    # CONTOH PENGGUNAAN - Edit koordinat sesuai lokasi pengujian
    # =========================================================

    # Tunggu koneksi
    node.get_logger().info('⏳ Menunggu koneksi FCU...')
    timeout = 30
    start = time.time()
    while not node.current_state.connected and time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=1.0)
        node.get_logger().info('...', throttle_duration_sec=5.0)

    if not node.current_state.connected:
        node.get_logger().error('❌ Tidak bisa terhubung ke FCU!')
        return

    node.get_logger().info('✅ FCU terhubung!')
    node.get_status()

    # ---------------------------------------------------------
    # CONTOH 1: AUTO MODE - Upload misi ke flight controller
    # Ganti koordinat dengan lokasi pengujian Sabtu
    # ---------------------------------------------------------
    # auto_waypoints = [
    #     {'lat': -6.890000, 'lon': 107.560000, 'alt': 0.0},  # WP1
    #     {'lat': -6.891000, 'lon': 107.561000, 'alt': 0.0},  # WP2
    #     {'lat': -6.892000, 'lon': 107.560000, 'alt': 0.0},  # WP3
    #     {'lat': -6.890000, 'lon': 107.559000, 'alt': 0.0},  # WP4 - kembali
    # ]
    # node.start_auto_mission(auto_waypoints)

    # ---------------------------------------------------------
    # CONTOH 2: GUIDED MODE - Navigasi dinamis
    # ---------------------------------------------------------
    # guided_waypoints = [
    #     {'lat': -6.890000, 'lon': 107.560000},
    #     {'lat': -6.891000, 'lon': 107.561000},
    # ]
    # node.guided_mission(guided_waypoints, acceptance_radius=3.0)

    # ---------------------------------------------------------
    # CONTOH 3: Override dari AUTO ke GUIDED
    # ---------------------------------------------------------
    # node.override_to_guided(-6.895000, 107.565000)
    # time.sleep(10)
    # node.return_to_auto()

    # Spin node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutdown...')
        node.set_mode('HOLD')
        node.disarm()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
