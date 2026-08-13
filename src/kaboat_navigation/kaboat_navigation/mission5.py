import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

from kaboat_navigation.nav_utils import (
    parse_gps_nav, bearing_deg, distance_m, WaypointLogger, cluster_scan, fuse_vision_lidar,
    GatePositionMemory, destination_point,
)
from kaboat_navigation.field_config import MISSION_TARGETS, TRANSIT_ARRIVAL_RADIUS_M


class GateNavigation(Node):
    """
    mission_5 - 항로추종 (게이트).
    빨강/초록 부표 쌍(게이트)을 순서대로 통과.

    센서 활용:
      - 카메라(camera/detections, camera_node.py 발행)는 색상 분류만 신뢰,
        실제 range/bearing은 LiDAR(/scan) 클러스터로 교체(fuse_vision_lidar)
      - GPS로 이미 통과한 게이트의 절대좌표를 기억(GatePositionMemory) ->
        회피기동으로 돌아서 같은 게이트를 다시 근접후보로 잡는 것을 방지

    가정
      - 'camera/detections' (String, JSON): camera_node.py가 발행하는 색상
        분류된 물체 목록. 실제 필드: [{"color":"R"/"G"/"B", "angle":라디안,
        "shape":..., "distance":거리(옵션)}, ...]
        게이트는 R/G만 쓰므로 buoys_cb에서 color를 'red'/'green'으로,
        angle(라디안)을 bearing_deg(도)로 변환해서 기존 로직 그대로 사용.

    로직:
      1. 전방 콘(±80도) 안의 red/green 각각 최근접 1개 -> LiDAR로 range/bearing 보정
      2. GPS로 게이트 중간점 절대좌표 계산 -> 이미 통과기록에 있으면 무시
      3. 게이트 중간 방위각 -> goal/heading 발행
      4. 게이트 중간점까지 거리가 최소값 찍고 다시 멀어지면 "통과" 판정
      5. 전방 콘 안에 게이트가 3초간 안 보이고 gate_count >= 1 이면 mission/done

    아직 게이트를 못 찾은 초반에는 m5s로 이동한다.
    """

    FORWARD_CONE_DEG = 80.0
    NO_GATE_TIMEOUT = 3.0
    MY_MISSION = 'mission_5'

    def __init__(self):
        super().__init__('mission5')
        self.active = False
        self.current_heading = None
        self.current_lat = None
        self.current_lon = None

        self.gate_count = 0
        self.tracking_min_dist = None
        self.last_gate_seen_time = None
        self.start_logged = False
        self.exiting = False
        self.latest_clusters = []
        self.gate_memory = GatePositionMemory()
        self.pending_gate_pos = None

        self.create_subscription(String, 'mission/active', self.active_cb, 10)
        self.create_subscription(String, 'mission/started', self.started_cb, 10)
        self.create_subscription(String, 'kaboat/gps_nav', self.gps_cb, 10)
        self.create_subscription(String, 'camera/detections', self.buoys_cb, 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_mission', 10)
        self.heading_pub = self.create_publisher(Float32, 'goal/heading', 10)
        self.done_pub = self.create_publisher(String, 'mission/done', 10)
        self.wp_logger = WaypointLogger(self, self.MY_MISSION)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('mission_5(항로추종/게이트) 노드 시작')

    def active_cb(self, msg):
        self.active = (msg.data == self.MY_MISSION)

    def started_cb(self, msg):
        if msg.data == self.MY_MISSION:
            self.gate_count = 0
            self.tracking_min_dist = None
            self.last_gate_seen_time = None
            self.start_logged = False
            self.exiting = False
            self.gate_memory = GatePositionMemory()
            self.pending_gate_pos = None
            self.get_logger().info('mission_5 시작 - 상태 초기화')

    def scan_cb(self, msg):
        n = len(msg.ranges)
        angles = [msg.angle_min + i * msg.angle_increment for i in range(n)]
        self.latest_clusters = cluster_scan(
            msg.ranges, angles, msg.range_min, msg.range_max, max_range=15.0)

    def gps_cb(self, msg):
        d = parse_gps_nav(msg.data)
        if 'imu_heading' in d:
            self.current_heading = d['imu_heading']
        if 'lat' in d:
            self.current_lat = d['lat']
        if 'lon' in d:
            self.current_lon = d['lon']

        if self.active and not self.start_logged and self.current_lat is not None:
            self.wp_logger.log('start', self.current_lat, self.current_lon, self.current_heading)
            self.start_logged = True

    # camera_node.py의 color 코드('R'/'G'/'B') -> 게이트 판정용 색상명.
    # 파랑(B)은 게이트와 무관하므로 매핑에서 제외 -> 자동으로 걸러짐.
    CAMERA_COLOR_MAP = {'R': 'red', 'G': 'green'}

    def buoys_cb(self, msg):
        if not self.active or self.exiting:
            return
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        # camera_node.py 실제 스키마({'color','angle'(rad),'distance'(옵션)})를
        # 기존 로직이 기대하던 형식({'color':'red'/'green','bearing_deg'(deg),
        # 'range_m'})으로 변환
        buoys = []
        for d in detections:
            color = self.CAMERA_COLOR_MAP.get(d.get('color'))
            if color is None:
                continue
            buoys.append({
                'color': color,
                'bearing_deg': math.degrees(d.get('angle', 0.0)),
                'range_m': d.get('distance'),
            })

        cone = self.FORWARD_CONE_DEG
        candidates = [b for b in buoys if abs(b['bearing_deg']) <= cone]
        fused = fuse_vision_lidar(candidates, self.latest_clusters, max_bearing_diff_deg=8.0)

        reds = [b for b in fused if b.get('color') == 'red']
        greens = [b for b in fused if b.get('color') == 'green']
        if not reds or not greens:
            return

        nearest_red = min(reds, key=lambda b: b['range_m'])
        nearest_green = min(greens, key=lambda b: b['range_m'])

        mid_bearing_rel = (nearest_red['bearing_deg'] + nearest_green['bearing_deg']) / 2.0
        mid_dist = (nearest_red['range_m'] + nearest_green['range_m']) / 2.0

        if self.current_lat is not None and self.current_heading is not None:
            abs_bearing = (self.current_heading + mid_bearing_rel) % 360.0
            gate_lat, gate_lon = destination_point(
                self.current_lat, self.current_lon, abs_bearing, mid_dist)
            if self.gate_memory.is_already_passed(gate_lat, gate_lon):
                return
            self.pending_gate_pos = (gate_lat, gate_lon)

        self.last_gate_seen_time = self.get_clock().now()

        if self.tracking_min_dist is None or mid_dist < self.tracking_min_dist:
            self.tracking_min_dist = mid_dist
        elif mid_dist > self.tracking_min_dist + 1.0:
            self.gate_count += 1
            self.get_logger().info(f'게이트 {self.gate_count}번 통과 추정')
            if self.pending_gate_pos is not None:
                self.gate_memory.mark_passed(*self.pending_gate_pos)
            self.tracking_min_dist = None

        if self.current_heading is not None:
            h_msg = Float32()
            h_msg.data = (self.current_heading + mid_bearing_rel) % 360.0
            self.heading_pub.publish(h_msg)

        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = max(-1.0, min(1.0, 0.02 * mid_bearing_rel))
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        if not self.active:
            return

        if self.exiting:
            self.handle_exit_transit()
            return

        if self.last_gate_seen_time is None:
            point = MISSION_TARGETS.get('m5s')
            if point is None:
                self.get_logger().warn('m5s 좌표 없음 (field_config.py 확인)', throttle_duration_sec=5.0)
                return
            if self.current_lat is not None:
                dist_to_point = distance_m(self.current_lat, self.current_lon, point[0], point[1])
                if dist_to_point > TRANSIT_ARRIVAL_RADIUS_M:
                    brg = bearing_deg(self.current_lat, self.current_lon, point[0], point[1])
                    h_msg = Float32()
                    h_msg.data = brg
                    self.heading_pub.publish(h_msg)
                    cmd = Twist()
                    cmd.linear.x = 0.2
                    self.cmd_pub.publish(cmd)
            return

        elapsed = (self.get_clock().now() - self.last_gate_seen_time).nanoseconds / 1e9
        if elapsed > self.NO_GATE_TIMEOUT and self.gate_count >= 1:
            self.get_logger().info(f'게이트 안 보임({elapsed:.1f}s) - 종료지점으로 이동 시작')
            self.exiting = True
            self.last_gate_seen_time = None

    def handle_exit_transit(self):
        point = MISSION_TARGETS.get('m5e')
        if point is not None and self.current_lat is not None:
            dist_to_point = distance_m(self.current_lat, self.current_lon, point[0], point[1])
            if dist_to_point > TRANSIT_ARRIVAL_RADIUS_M:
                brg = bearing_deg(self.current_lat, self.current_lon, point[0], point[1])
                h_msg = Float32()
                h_msg.data = brg
                self.heading_pub.publish(h_msg)
                cmd = Twist()
                cmd.linear.x = 0.2
                self.cmd_pub.publish(cmd)
                return

        self.get_logger().info('mission_5 완료')
        self.wp_logger.log('end', self.current_lat, self.current_lon, self.current_heading)
        done = String()
        done.data = self.MY_MISSION
        self.done_pub.publish(done)


def main(args=None):
    rclpy.init(args=args)
    node = GateNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
