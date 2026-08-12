import threading
import math

import rclpy
from rclpy.node import Node

from flask import Flask, render_template_string
from flask_socketio import SocketIO

from smc_3000_msgs.msg import Nmea, DrpvaA


GPS_TOPIC = "/smc3000/gngga"
DRPVAA_TOPIC = "/smc3000/drpva"


app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")


HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>SMC3000 GPS Target Map</title>

    <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css">
    <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
    <script src="https://cdn.socket.io/4.7.2/socket.io.min.js"></script>

    <style>
        body {
            margin: 0;
            font-family: Arial, sans-serif;
        }

        #map {
            width: 100vw;
            height: 100vh;
        }

        #info {
            position: absolute;
            top: 10px;
            left: 10px;
            z-index: 999;
            background: white;
            padding: 12px;
            border-radius: 8px;
            font-size: 14px;
            box-shadow: 0 0 8px rgba(0,0,0,0.3);
            line-height: 1.35;
            max-width: 310px;
        }

        #status_box {
            margin-top: 6px;
            font-weight: bold;
        }

        .section-title {
            font-weight: bold;
            margin-top: 8px;
            border-top: 1px solid #ddd;
            padding-top: 6px;
        }
    </style>
</head>

<body>
    <div id="info">
        <b>SMC3000 GPS Target Map</b><br>

        <div class="section-title">Current GPS</div>
        Lat: <span id="lat">----</span><br>
        Lon: <span id="lon">----</span><br>
        Alt: <span id="alt">----</span> m<br>
        Fix Quality: <span id="fix">----</span><br>
        Satellites: <span id="sat">----</span><br>
        Points: <span id="points">0</span><br>

        <div class="section-title">Attitude</div>
        Heading: <span id="heading">----</span> deg<br>
        Pitch: <span id="pitch">----</span> deg<br>
        Roll: <span id="roll">----</span> deg<br>
        INS: <span id="ins_status">----</span><br>

        <div class="section-title">Clicked Target</div>
        Target Lat: <span id="target_lat">----</span><br>
        Target Lon: <span id="target_lon">----</span><br>
        Distance: <span id="target_dist">----</span> m<br>
        Bearing: <span id="target_bearing">----</span> deg<br>
        Heading Error: <span id="heading_error">----</span> deg<br>

        <div id="status_box">Waiting GPS data...</div>
    </div>

    <div id="map"></div>

    <script>
        var map = L.map('map').setView([35.1796, 129.0756], 17);

        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 22,
            attribution: '© OpenStreetMap'
        }).addTo(map);

        var currentMarker = null;
        var targetMarkers = []; // 여러 목표 지점을 담을 배열
        var targetLine = null;

        function createBoatIcon(heading) {
            return L.divIcon({
                className: "boat-icon",
                html: `
                <div style="
                    transform: rotate(${heading || 0}deg);
                    width:40px;
                    height:60px;
                    display:flex;
                    justify-content:center;
                    align-items:center;
                ">
                    <div style="
                        width:22px;
                        height:50px;
                        background:#1976d2;
                        border-radius:50% 50% 15% 15%;
                        border:2px solid white;
                        box-shadow:0 0 5px black;
                        position:relative;
                    ">
                        <div style="
                            position:absolute;
                            top:-3px;
                            left:50%;
                            transform:translateX(-50%);
                            width:0;
                            height:0;
                            border-left:11px solid transparent;
                            border-right:11px solid transparent;
                            border-bottom:15px solid white;
                        ">
                        </div>
                        <div style="
                            position:absolute;
                            top:18px;
                            left:50%;
                            transform:translateX(-50%);
                            width:8px;
                            height:15px;
                            background:white;
                        ">
                        </div>
                    </div>
                </div>
                `,
                iconSize:[40,60],
                iconAnchor:[20,30]
            });
        }

        var path = [];
        var polyline = L.polyline(path).addTo(map);

        var firstFix = true;

        var currentLat = null;
        var currentLon = null;
        var currentHeading = null;

        var socket = io();

        function toRad(deg) {
            return deg * Math.PI / 180.0;
        }

        function toDeg(rad) {
            return rad * 180.0 / Math.PI;
        }

        function distanceMeters(lat1, lon1, lat2, lon2) {
            var R = 6371000.0;

            var phi1 = toRad(lat1);
            var phi2 = toRad(lat2);
            var dphi = toRad(lat2 - lat1);
            var dlambda = toRad(lon2 - lon1);

            var a = Math.sin(dphi / 2.0) * Math.sin(dphi / 2.0) +
                    Math.cos(phi1) * Math.cos(phi2) *
                    Math.sin(dlambda / 2.0) * Math.sin(dlambda / 2.0);

            var c = 2.0 * Math.atan2(Math.sqrt(a), Math.sqrt(1.0 - a));

            return R * c;
        }

        function bearingDeg(lat1, lon1, lat2, lon2) {
            var phi1 = toRad(lat1);
            var phi2 = toRad(lat2);
            var dlambda = toRad(lon2 - lon1);

            var y = Math.sin(dlambda) * Math.cos(phi2);
            var x = Math.cos(phi1) * Math.sin(phi2) -
                    Math.sin(phi1) * Math.cos(phi2) * Math.cos(dlambda);

            var brng = toDeg(Math.atan2(y, x));
            brng = (brng + 360.0) % 360.0;

            return brng;
        }

        function headingErrorDeg(targetBearing, heading) {
            if (heading === null) {
                return null;
            }

            var error = (targetBearing - heading + 540.0) % 360.0 - 180.0;
            return error;
        }

        function updateTargetInfo() {
            if (targetLine !== null) {
                map.removeLayer(targetLine);
                targetLine = null;
            }

            // 마커가 없거나 현재 위치가 없으면 UI 초기화
            if (currentLat === null || currentLon === null || targetMarkers.length === 0) {
                document.getElementById("target_lat").innerText = "----";
                document.getElementById("target_lon").innerText = "----";
                document.getElementById("target_dist").innerText = "----";
                document.getElementById("target_bearing").innerText = "----";
                document.getElementById("heading_error").innerText = "----";
                return;
            }

            // 항상 배열의 첫 번째 마커(가장 가까운/먼저 찍은 목표)를 기준으로 계산
            var firstTarget = targetMarkers[0].getLatLng();
            var tLat = firstTarget.lat;
            var tLon = firstTarget.lng;

            var dist = distanceMeters(currentLat, currentLon, tLat, tLon);
            var brng = bearingDeg(currentLat, currentLon, tLat, tLon);
            var err = headingErrorDeg(brng, currentHeading);

            document.getElementById("target_lat").innerText = tLat.toFixed(8);
            document.getElementById("target_lon").innerText = tLon.toFixed(8);
            document.getElementById("target_dist").innerText = dist.toFixed(2);
            document.getElementById("target_bearing").innerText = brng.toFixed(2);

            if (err === null) {
                document.getElementById("heading_error").innerText = "----";
            } else {
                document.getElementById("heading_error").innerText = err.toFixed(2);
            }

            // 내 위치부터 모든 목표물들을 순서대로 빨간 점선으로 연결
            var linePoints = [[currentLat, currentLon]];
            for (var i = 0; i < targetMarkers.length; i++) {
                linePoints.push(targetMarkers[i].getLatLng());
            }

            targetLine = L.polyline(linePoints, {
                dashArray: "8, 8",
                color: "red"
            }).addTo(map);
        }

        map.on('click', function(e) {
            // 새로운 마커 생성 및 지도에 추가
            var newMarker = L.marker([e.latlng.lat, e.latlng.lng]).addTo(map);
            
            newMarker.bindTooltip("클릭 시 핀 삭제", {direction: 'top'});

            // 생성된 핀을 클릭했을 때 지워지는 이벤트 등록
            newMarker.on('click', function() {
                map.removeLayer(newMarker); // 지도에서 마커 삭제
                targetMarkers = targetMarkers.filter(m => m !== newMarker); // 배열에서 마커 제외
                updateTargetInfo(); // 정보 및 선 갱신
            });

            targetMarkers.push(newMarker);
            updateTargetInfo();

            console.log("Clicked target:", e.latlng.lat, e.latlng.lng);
        });

        socket.on('gps', function(data) {
            if (data.lat === null || data.lon === null) {
                return;
            }

            currentLat = data.lat;
            currentLon = data.lon;

            document.getElementById("lat").innerText = currentLat.toFixed(8);
            document.getElementById("lon").innerText = currentLon.toFixed(8);

            if (data.alt !== null) {
                document.getElementById("alt").innerText = data.alt.toFixed(2);
            } else {
                document.getElementById("alt").innerText = "----";
            }

            document.getElementById("fix").innerText = data.fix;
            document.getElementById("sat").innerText = data.sat;
            document.getElementById("status_box").innerText = "GPS data receiving";

            var pos = [currentLat, currentLon];

            if (currentMarker === null) {
                currentMarker = L.marker(
                    pos,
                    {
                        icon:createBoatIcon(currentHeading)
                    }
                ).addTo(map);
            } else {
                currentMarker.setLatLng(pos);
            }

            path.push(pos);
            polyline.setLatLngs(path);

            document.getElementById("points").innerText = path.length;

            if (firstFix) {
                map.setView(pos, 19);
                firstFix = false;
            } else {
                map.panTo(pos);
            }

            updateTargetInfo();
        });

        socket.on('attitude', function(data) {
            if (data.heading !== null) {
                currentHeading = data.heading;
                document.getElementById("heading").innerText = data.heading.toFixed(2);

                if(currentMarker !== null)
                {
                    currentMarker.setIcon(
                        createBoatIcon(currentHeading)
                    );
                }
            }

            if (data.pitch !== null) {
                document.getElementById("pitch").innerText = data.pitch.toFixed(2);
            }

            if (data.roll !== null) {
                document.getElementById("roll").innerText = data.roll.toFixed(2);
            }

            if (data.ins_status !== null) {
                document.getElementById("ins_status").innerText = data.ins_status;
            }

            updateTargetInfo();
        });
    </script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(HTML_PAGE)


def nmea_degmin_to_decimal(value, direction):
    if value is None:
        return None

    if value == 0.0:
        return None

    raw = float(value)

    degrees = int(raw // 100)
    minutes = raw - degrees * 100.0

    decimal = degrees + minutes / 60.0

    if direction in ["S", "W"]:
        decimal *= -1.0

    return decimal


def normalize_heading(value):
    if value is None:
        return None

    heading = float(value)

    while heading < 0.0:
        heading += 360.0

    while heading >= 360.0:
        heading -= 360.0

    return heading


class GPSWebMapNode(Node):
    def __init__(self):
        super().__init__("gps_web_map_node")

        self.create_subscription(
            Nmea,
            GPS_TOPIC,
            self.gps_callback,
            10
        )

        self.create_subscription(
            DrpvaA,
            DRPVAA_TOPIC,
            self.drpvaa_callback,
            10
        )

        self.get_logger().info("GPS Web Map Node Started")
        self.get_logger().info(f"Subscribing GPS topic: {GPS_TOPIC}")
        self.get_logger().info("GPS message type: smc_3000_msgs/msg/Nmea")
        self.get_logger().info(f"Subscribing DRPVAA topic: {DRPVAA_TOPIC}")
        self.get_logger().info("DRPVAA message type: smc_3000_msgs/msg/DrpvaA")

    def gps_callback(self, msg):
        lat = nmea_degmin_to_decimal(
            msg.latitude,
            msg.latitude_direction
        )

        lon = nmea_degmin_to_decimal(
            msg.longitude,
            msg.longitude_direction
        )

        if lat is None or lon is None:
            return

        gps_data = {
            "lat": lat,
            "lon": lon,
            "alt": float(msg.altitude),
            "fix": int(msg.fix_quality),
            "sat": int(msg.num_satellites),
        }

        socketio.emit("gps", gps_data)

    def drpvaa_callback(self, msg):
        heading = normalize_heading(msg.heading)
        pitch = float(msg.pitch)
        roll = float(msg.roll)

        attitude_data = {
            "heading": heading,
            "pitch": pitch,
            "roll": roll,
            "ins_status": str(msg.sol_status),
        }

        socketio.emit("attitude", attitude_data)


def ros_spin_thread():
    rclpy.init()
    node = GPSWebMapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


def main():
    ros_thread = threading.Thread(target=ros_spin_thread)
    ros_thread.daemon = True
    ros_thread.start()

    print("======================================")
    print("SMC3000 GPS Target Map Server Started")
    print("Open browser:")
    print("http://localhost:5000")
    print("======================================")

    socketio.run(
        app,
        host="0.0.0.0",
        port=5000,
    )


if __name__ == "__main__":
    main()
