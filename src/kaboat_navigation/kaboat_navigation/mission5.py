import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

from kaboat_navigation.field_config import MISSION_TARGETS, TRANSIT_ARRIVAL_RADIUS_M


def bearing_deg(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def distance_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def destination_point(lat, lon, bearing_deg_val, dist_m):
    """시작좌표 + 방위각(도) + 거리(m)로 목적지 좌표 계산 (구면 공식)."""
    R = 6371000.0
    phi1 = math.radians(lat)
    lam1 = math.radians(lon)
    theta = math.radians(bearing_deg_val)
    delta = dist_m / R

    phi2 = math.asin(
        math.sin(phi1) * math.cos(delta) +
        math.cos(phi1) * math.sin(delta) * math.cos(theta)
    )
    lam2 = lam1 + math.atan2(
        math.sin(theta) * math.sin(delta) * math.cos(phi1),
        math.cos(delta) - math.sin(phi1) * math.sin(phi2)
    )
    return math.degrees(phi2), math.degrees(lam2)


def cluster_scan(ranges, angles, range_min, range_max, max_range, jump_threshold=0.3):
    """연속된 유효 거리값들을 클러스터로 묶는다.
    각 클러스터의 대표각도(중앙 인덱스 각도)와 평균거리를 반환."""
    n = len(ranges)
    effective_max = min(range_max, max_range)
    valid = [r if range_min < r < effective_max else None for r in ranges]

    clusters = []
    start_idx = None
    for i in range(n):
        if valid[i] is not None:
            if start_idx is None:
                start_idx = i
            elif abs(valid[i] - valid[i - 1]) > jump_threshold:
                clusters.append((start_idx, i - 1))
                start_idx = i
        else:
            if start_idx is not None:
                clusters.append((start_idx, i - 1))
                start_idx = None
    if start_idx is not None:
        clusters.append((start_idx, n - 1))

    result = []
    for s, e in clusters:
        seg = [valid[i] for i in range(s, e + 1) if valid[i] is not None]
        if not seg:
            continue
        center_idx = (s + e) // 2
        result.append({
            'center_angle': angles[center_idx],
            'min_range': sum(seg) / len(seg),
        })
    return result


def fuse_vision_lidar(candidates, clusters, max_bearing_diff_deg=8.0):
    """카메라 candidate(색상 + bearing_deg 힌트)를 라이다 클러스터와 매칭해서
    range/bearing을 라이다 기준값으로 교체. 매칭되는 클러스터가 없으면
    (라이다로 확인 안 된 값은 신뢰하지 않음) 그 candidate는 버림."""
    fused = []
    for cand in candidates:
        best = None
        best_diff = max_bearing_diff_deg
        for cluster in clusters:
            cluster_deg = math.degrees(cluster['center_angle'])
            diff = abs(cluster_deg - cand['bearing_deg'])
            if diff < best_diff:
                best_diff = diff
                best = cluster
        if best is not None:
            fused.append({
                'color': cand['color'],
                'bearing_deg': math.degrees(best['center_angle']),
                'range_m': best['min_range'],
            })
    return fused


class GatePositionMemory:
    """이미 통과한 게이트의 절대좌표(lat, lon)를 기억해서, 회피기동으로
    돌아섰다가 다시 같은 게이트를 근접후보로 잡아 중복 카운트하는 것을 방지."""

    SAME_GATE_RADIUS_M = 3.0

    def __init__(self):
        self.passed_positions = []  # [(lat, lon), ...]

    def is_already_passed(self, lat, lon):
        for plat, plon in self.passed_positions:
            if distance_m(plat, plon, lat, lon) <= self.SAME_GATE_RADIUS_M:
                return True
        return False

    def mark_passed(self, lat, lon):
        self.passed_positions.append((lat, lon))


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

    # camera_node.py의 color 코드('R'/'G'/'B') -> 게이트 판정용 색상명.
    # 파랑(B)은 게이트와 무관하므로 매핑에서 제외 -> 자동으로 걸러짐.
    CAMERA_COLOR_MAP = {'R': 'red', 'G': 'green'}

    def __init__(self):
        super().__init__('mission5')
        self.active = False
        self.current_heading = None
        self.current_lat = None
        self.current_lon = None

        self.gate_count = 0
        self.tracking_min_dist = None
        self.last_gate_seen_time = None
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

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('mission_5(항로추종/게이트) 노드 시작')

    def active_cb(self, msg):
        self.active = (msg.data == self.MY_MISSION)

    def started_cb(self, msg):
        if msg.data == self.MY_MISSION:
            self.gate_count = 0
            self.tracking_min_dist = None
            self.last_gate_seen_time = None
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
        try:
            for part in msg.data.split(','):
                if part.startswith('lat='):
                    self.current_lat = float(part.split('=')[1])
                elif part.startswith('lon='):
                    self.current_lon = float(part.split('=')[1])
                elif part.startswith('imu_heading='):
                    self.current_heading = float(part.split('=')[1])
        except (ValueError, IndexError):
            pass

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
