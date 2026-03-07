#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from flask import Flask, render_template_string, request, jsonify
import threading
import math

app = Flask(__name__)

# Global variables
current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'heading': 0.0}
waypoints = []
mission_active = False
current_waypoint_index = 0
ros_node = None

# Simple HTML template embedded in Python
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>USV Dashboard</title>
    <meta charset="utf-8">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: Arial, sans-serif; background: #1a1a2e; color: white; }
        .header { background: #16213e; padding: 20px; text-align: center; border-bottom: 3px solid #00adb5; }
        .header h1 { margin: 0; color: #00adb5; }
        .container { display: flex; height: calc(100vh - 80px); }
        #map { flex: 1; }
        .sidebar { width: 350px; background: #16213e; padding: 20px; overflow-y: auto; }
        .section { margin-bottom: 25px; }
        .section h2 { color: #00adb5; border-bottom: 2px solid #0f3460; padding-bottom: 10px; margin-bottom: 15px; font-size: 1.2em; }
        .telemetry-item { display: flex; justify-content: space-between; padding: 10px; border-bottom: 1px solid #0f3460; }
        .telemetry-label { color: #aaa; }
        .telemetry-value { color: #00adb5; font-weight: bold; }
        .btn { width: 100%; padding: 12px; margin: 5px 0; border: none; border-radius: 5px; cursor: pointer; font-size: 14px; font-weight: bold; transition: 0.3s; }
        .btn:hover { opacity: 0.8; }
        .btn-primary { background: #00adb5; color: white; }
        .btn-success { background: #2ecc71; color: white; }
        .btn-danger { background: #e74c3c; color: white; }
        input { width: 100%; padding: 10px; margin: 5px 0; background: #0f3460; border: 1px solid #00adb5; color: white; border-radius: 5px; }
        .waypoint-list { max-height: 200px; overflow-y: auto; margin-top: 10px; }
        .waypoint-item { background: #0f3460; padding: 10px; margin: 5px 0; border-radius: 5px; display: flex; justify-content: space-between; align-items: center; }
        .waypoint-item button { background: #e74c3c; border: none; color: white; padding: 5px 10px; border-radius: 3px; cursor: pointer; }
        .status-indicator { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 5px; }
        .status-active { background: #2ecc71; box-shadow: 0 0 5px #2ecc71; }
        .status-idle { background: #95a5a6; }
    </style>
</head>
<body>
    <div class="header">
        <h1>🚢 USV Mission Control Dashboard</h1>
    </div>
    <div class="container">
        <div id="map"></div>
        <div class="sidebar">
            <div class="section">
                <h2>📊 Telemetry</h2>
                <div class="telemetry-item">
                    <span class="telemetry-label">Status:</span>
                    <span class="telemetry-value">
                        <span id="status-dot" class="status-indicator status-idle"></span>
                        <span id="status">Idle</span>
                    </span>
                </div>
                <div class="telemetry-item">
                    <span class="telemetry-label">Position X:</span>
                    <span class="telemetry-value" id="pos-x">0.00 m</span>
                </div>
                <div class="telemetry-item">
                    <span class="telemetry-label">Position Y:</span>
                    <span class="telemetry-value" id="pos-y">0.00 m</span>
                </div>
                <div class="telemetry-item">
                    <span class="telemetry-label">Heading:</span>
                    <span class="telemetry-value" id="heading">0.0°</span>
                </div>
                <div class="telemetry-item">
                    <span class="telemetry-label">Waypoint:</span>
                    <span class="telemetry-value" id="waypoint-progress">0 / 0</span>
                </div>
            </div>
            
            <div class="section">
                <h2>📍 Mission Planning</h2>
                <input type="number" id="wp-x" placeholder="X Position (meters)" step="0.1">
                <input type="number" id="wp-y" placeholder="Y Position (meters)" step="0.1">
                <button class="btn btn-primary" onclick="addWaypoint()">➕ Add Waypoint</button>
                <div class="waypoint-list" id="waypoint-list"></div>
                <button class="btn btn-success" onclick="uploadMission()">📤 Upload Mission</button>
                <button class="btn btn-primary" onclick="clearWaypoints()">🗑️ Clear All</button>
            </div>
            
            <div class="section">
                <h2>🎮 Mission Control</h2>
                <button class="btn btn-success" onclick="startMission()">▶️ Start Mission</button>
                <button class="btn btn-danger" onclick="stopMission()">⏹️ Stop Mission</button>
            </div>
        </div>
    </div>
    
    <script>
        // Initialize map (centered at origin)
        const map = L.map('map').setView([0, 0], 16);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap'
        }).addTo(map);
        
        // Convert meters to lat/lon (approximate)
        function metersToLatLon(x, y) {
            const lat = y / 111320;
            const lon = x / (111320 * Math.cos(0));
            return [lat, lon];
        }
        
        // USV marker
        const usvMarker = L.marker([0, 0], {
            icon: L.divIcon({
                html: '<div style="background: #00adb5; width: 20px; height: 20px; border-radius: 50%; border: 3px solid white;"></div>',
                iconSize: [20, 20],
                className: ''
            })
        }).addTo(map);
        
        let waypoints = [];
        let waypointMarkers = [];
        
        // Update telemetry every 500ms
        setInterval(async () => {
            const res = await fetch('/api/telemetry');
            const data = await res.json();
            
            document.getElementById('pos-x').textContent = data.x.toFixed(2) + ' m';
            document.getElementById('pos-y').textContent = data.y.toFixed(2) + ' m';
            document.getElementById('heading').textContent = data.heading.toFixed(1) + '°';
            document.getElementById('waypoint-progress').textContent = data.waypoint_index + ' / ' + data.total_waypoints;
            
            const statusDot = document.getElementById('status-dot');
            const statusText = document.getElementById('status');
            if (data.mission_active) {
                statusDot.className = 'status-indicator status-active';
                statusText.textContent = 'Active';
            } else {
                statusDot.className = 'status-indicator status-idle';
                statusText.textContent = 'Idle';
            }
            
            // Update USV position on map
            const [lat, lon] = metersToLatLon(data.x, data.y);
            usvMarker.setLatLng([lat, lon]);
            map.setView([lat, lon], map.getZoom());
        }, 500);
        
        // Click on map to add waypoint
        map.on('click', (e) => {
            const lat = e.latlng.lat;
            const lon = e.latlng.lng;
            const y = lat * 111320;
            const x = lon * 111320 * Math.cos(0);
            document.getElementById('wp-x').value = x.toFixed(2);
            document.getElementById('wp-y').value = y.toFixed(2);
        });
        
        function addWaypoint() {
            const x = parseFloat(document.getElementById('wp-x').value);
            const y = parseFloat(document.getElementById('wp-y').value);
            
            if (isNaN(x) || isNaN(y)) {
                alert('Please enter valid coordinates');
                return;
            }
            
            waypoints.push({x, y});
            
            // Add marker to map
            const [lat, lon] = metersToLatLon(x, y);
            const marker = L.marker([lat, lon], {
                icon: L.divIcon({
                    html: '<div style="background: #e74c3c; width: 15px; height: 15px; border-radius: 50%; border: 2px solid white;"></div>',
                    iconSize: [15, 15],
                    className: ''
                })
            }).addTo(map);
            waypointMarkers.push(marker);
            
            updateWaypointList();
            document.getElementById('wp-x').value = '';
            document.getElementById('wp-y').value = '';
        }
        
        function updateWaypointList() {
            const list = document.getElementById('waypoint-list');
            list.innerHTML = waypoints.map((wp, i) => 
                `<div class="waypoint-item">
                    <span>WP${i+1}: X=${wp.x.toFixed(2)}, Y=${wp.y.toFixed(2)}</span>
                    <button onclick="removeWaypoint(${i})">✕</button>
                </div>`
            ).join('');
        }
        
        function removeWaypoint(index) {
            waypoints.splice(index, 1);
            map.removeLayer(waypointMarkers[index]);
            waypointMarkers.splice(index, 1);
            updateWaypointList();
        }
        
        function clearWaypoints() {
            waypoints = [];
            waypointMarkers.forEach(m => map.removeLayer(m));
            waypointMarkers = [];
            updateWaypointList();
        }
        
        async function uploadMission() {
            if (waypoints.length === 0) {
                alert('Please add waypoints first');
                return;
            }
            
            const res = await fetch('/api/mission/upload', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({waypoints})
            });
            const data = await res.json();
            alert(`Mission uploaded: ${data.count} waypoints`);
        }
        
        async function startMission() {
            const res = await fetch('/api/mission/start', {method: 'POST'});
            const data = await res.json();
            if (data.status === 'success') {
                alert('Mission started!');
            }
        }
        
        async function stopMission() {
            const res = await fetch('/api/mission/stop', {method: 'POST'});
            const data = await res.json();
            if (data.status === 'success') {
                alert('Mission stopped!');
            }
        }
    </script>
</body>
</html>
'''

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/telemetry')
def telemetry():
    return jsonify({
        'x': current_position['x'],
        'y': current_position['y'],
        'z': current_position['z'],
        'heading': current_position['heading'],
        'mission_active': mission_active,
        'waypoint_index': current_waypoint_index,
        'total_waypoints': len(waypoints)
    })

@app.route('/api/mission/upload', methods=['POST'])
def upload_mission():
    global waypoints, current_waypoint_index
    data = request.get_json()
    waypoints = data.get('waypoints', [])
    current_waypoint_index = 0
    if ros_node:
        ros_node.get_logger().info(f'Mission uploaded: {len(waypoints)} waypoints')
    return jsonify({'status': 'success', 'count': len(waypoints)})

@app.route('/api/mission/start', methods=['POST'])
def start_mission():
    global mission_active, current_waypoint_index
    if len(waypoints) == 0:
        return jsonify({'status': 'error', 'message': 'No waypoints'})
    mission_active = True
    current_waypoint_index = 0
    if ros_node:
        ros_node.get_logger().info('Mission started!')
    return jsonify({'status': 'success'})

@app.route('/api/mission/stop', methods=['POST'])
def stop_mission():
    global mission_active
    mission_active = False
    if ros_node:
        ros_node.get_logger().info('Mission stopped!')
        # Stop thrusters via gz command
        import subprocess
        subprocess.run(['gz', 'topic', '-t', '/model/wam-v/joint/left_propeller_joint/cmd_thrust', 
                       '-m', 'gz.msgs.Double', '-p', 'data: 0.0'],
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(['gz', 'topic', '-t', '/model/wam-v/joint/right_propeller_joint/cmd_thrust', 
                       '-m', 'gz.msgs.Double', '-p', 'data: 0.0'],
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return jsonify({'status': 'success'})

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        
        # NOTE: Tidak bisa publish langsung ke Gazebo topic dengan ROS2 karena nama topic invalid
        # Kita akan gunakan gz command atau bridge
        
        # Subscribe to pose (if available)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/wamv/pose',
            self.pose_callback,
            10
        )
        
        # Mission execution timer
        self.mission_timer = self.create_timer(0.1, self.execute_mission)
        
        self.get_logger().info('🌐 Dashboard server running at http://localhost:5000')
        self.get_logger().info('📡 Waiting for pose data...')
        self.get_logger().info('⚠️  Make sure ros_gz_bridge is running!')
    
    def pose_callback(self, msg):
        global current_position
        current_position['x'] = msg.pose.position.x
        current_position['y'] = msg.pose.position.y
        current_position['z'] = msg.pose.position.z
        
        # Calculate heading from quaternion (simplified)
        current_position['heading'] = 0.0
    
    def execute_mission(self):
        global mission_active, current_waypoint_index, waypoints
        
        if not mission_active or not waypoints:
            return
        
        if current_waypoint_index >= len(waypoints):
            mission_active = False
            self.get_logger().info('✅ Mission completed!')
            # Stop thrusters via gz command
            import subprocess
            subprocess.run(['gz', 'topic', '-t', '/model/wam-v/joint/left_propeller_joint/cmd_thrust', '-m', 'gz.msgs.Double', '-p', 'data: 0.0'], 
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(['gz', 'topic', '-t', '/model/wam-v/joint/right_propeller_joint/cmd_thrust', '-m', 'gz.msgs.Double', '-p', 'data: 0.0'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return
        
        # Get current waypoint
        wp = waypoints[current_waypoint_index]
        
        # Calculate distance to waypoint
        dx = wp['x'] - current_position['x']
        dy = wp['y'] - current_position['y']
        distance = math.sqrt(dx**2 + dy**2)
        
        # Waypoint reached threshold
        if distance < 2.0:  # 2 meters
            self.get_logger().info(f'✅ Waypoint {current_waypoint_index + 1} reached!')
            current_waypoint_index += 1
            return
        
        # Simple proportional navigation
        base_thrust = min(distance * 10.0, 100.0)  # Max 100
        
        # Publish thrust via gz command
        self.publish_thrust_gz(base_thrust, base_thrust)
    
    def publish_thrust_gz(self, left, right):
        """Publish thrust directly to Gazebo using gz command"""
        import subprocess
        subprocess.Popen(['gz', 'topic', '-t', '/model/wam-v/joint/left_propeller_joint/cmd_thrust', 
                         '-m', 'gz.msgs.Double', '-p', f'data: {left}'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.Popen(['gz', 'topic', '-t', '/model/wam-v/joint/right_propeller_joint/cmd_thrust', 
                         '-m', 'gz.msgs.Double', '-p', f'data: {right}'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def main(args=None):
    global ros_node
    
    rclpy.init(args=args)
    ros_node = DashboardNode()
    
    # Start Flask in separate thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
