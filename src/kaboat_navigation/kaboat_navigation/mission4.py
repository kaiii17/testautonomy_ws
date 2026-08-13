import math
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

from kaboat_navigation.field_config import MISSION_TARGETS, MISSION_TARGETS_CONFIG


class Mission4(Node):
    """
    미션 4 - 탐색/선회.
    대회 규정: 지정된 색 부표를 중심으로 360도 완전히 선회해야 완료. 회전방향은
    색상별 고정 - 빨강/초록=시계방향(CW), 흰색=반시계방향(CCW). 모양은 무관.

    흐름:
      MOVING : m4s로 GPS 이동. 도착하면 SEARCH로 전환.
      SEARCH : camera/detections에서 TARGET_COLOR가 연속으로 CONFIRM_STREAK
               프레임 이상 잡히면 확정, APPROACH로 전환. 확정 순간의 카메라
               각도를 기억해둔다.
      APPROACH: 카메라 각도는 방향 힌트로만 쓰고, 라이다가 그 각도 근처의
                실제 클러스터를 찾아서 방향/거리를 확정한다. 라이다가 확정한
                거리가 CIRCLE_START_DIST 이내가 되면 CIRCLE로.
      CIRCLE : 라이다만 사용. 가장 가까운 클러스터를 부표로 보고 거리
               유지하며 원선회. imu_heading을 부호 포함으로 누적, 360도
               채우면 EXIT로.
      EXIT   : m4e로 GPS 이동. 도착하면 done 발행.

    목표색은 field_config.py의 MISSION_TARGETS_CONFIG에서 가져옴.
    """

    MY_MISSION = 'mission_4'

    TARGET_COLOR = MISSION_TARGETS_CONFIG['mission_4']['color']

    ARRIVAL_RADIUS = 1.0
    FINISH_RADIUS = 1.0
    CONFIRM_STREAK = 5

    CIRCLE_START_DIST = 3.0
    ORBIT_RADIUS = 3.0
    CLUSTER_JUMP_THRESHOLD = 0.3
    ANGLE_MATCH_TOLERANCE = 0.3

    CRUISE_SPEED = 0.3
    K_ANGLE = 0.8
    KP_ORBIT = 0.05

    ROTATE_CW = {'R': True, 'G': True, 'W': False}

    def __init__(self):
        super().__init__('mission_4')

        self.active = False
        self.phase = 'MOVING'   # MOVING / SEARCH / APPROACH / CIRCLE / EXIT

        self.current_lat = None
        self.current_lon = None
        self.current_heading = None

        self.confirm_streak = 0
        self.cam_angle = None

        self.buoy_dist = None
        self.buoy_angle = None

        self.turn_cw = True
        self.last_heading = None
        self.accumulated_turn_deg = 0.0

        self.create_subscription(String, 'mission/active', self.active_cb, 10)
        self.create_subscription(String, 'mission/started', self.started_cb, 10)
        self.create_subscription(String, 'camera/detections', self.detections_cb, 10)
        self.create_subscription(String, 'kaboat/gps_nav', self.gps_cb, 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_mission', 10)
        self.done_pub = self.create_publisher(String, 'mission/done', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('미션4(탐색) 노드 대기 중')

    def active_cb(self, msg):
        self.active = (msg.data == self.MY_MISSION)

    def started_cb(self, msg):
        if msg.data == self.MY_MISSION:
            self.get_logger().info('★ 미션4 시작 (MOVING -> m4s로 이동) ★')
            self.phase = 'MOVING'
            self.confirm_streak = 0
            self.cam_angle = None
            self.accumulated_turn_deg = 0.0
            self.last_heading = None

    def gps_cb(self, msg):
        try:
            for part in msg.data.split(','):
                if part.startswith('lat='):
                    self.current_lat = float(part.split('=')[1])
                elif part.startswith('lon='):
                    self.current_lon = float(part.split('=')[1])
                elif part.startswith('imu_heading='):
                    heading = float(part.split('=')[1])
                    self.current_heading = heading
                    if self.phase == 'CIRCLE':
                        self.accumulate_turn(heading)
        except (ValueError, IndexError):
            pass

    def accumulate_turn(self, heading):
        if self.last_heading is not None:
            delta = heading - self.last_heading
            if delta > 180:
                delta -= 360
            elif delta < -180:
                delta += 360
            self.accumulated_turn_deg += delta if self.turn_cw else -delta
        self.last_heading = heading

    def detections_cb(self, msg):
        if not self.active or self.phase != 'SEARCH':
            return
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        match = next((d for d in detections if d.get('color') == self.TARGET_COLOR), None)

        if match is not None:
            self.confirm_streak += 1
            self.cam_angle = match['angle']
        else:
            self.confirm_streak = 0

        if self.confirm_streak >= self.CONFIRM_STREAK:
            self.get_logger().info(f'목표색({self.TARGET_COLOR}) 확정 → 접근 시작')
            self.turn_cw = self.ROTATE_CW.get(self.TARGET_COLOR, True)
            self.phase = 'APPROACH'

    def scan_cb(self, msg):
        if not self.active:
            return
        if self.phase == 'APPROACH':
            self.match_cluster_to_camera_angle(msg)
        elif self.phase == 'CIRCLE':
            self.detect_nearest_buoy(msg)

    def match_cluster_to_camera_angle(self, msg):
        if self.cam_angle is None:
            self.buoy_dist, self.buoy_angle = None, None
            return

        n = len(msg.ranges)
        ranges = [r if msg.range_min < r < msg.range_max else float('inf') for r in msg.ranges]

        clusters = []
        i = 0
        while i < n:
            if ranges[i] < float('inf'):
                start = i
                while (i + 1 < n and ranges[i + 1] < float('inf')
                       and abs(ranges[i + 1] - ranges[i]) < self.CLUSTER_JUMP_THRESHOLD):
                    i += 1
                clusters.append((start, i))
            i += 1

        best, best_diff = None, self.ANGLE_MATCH_TOLERANCE
        for start, end in clusters:
            center_idx = (start + end) // 2
            center_angle = msg.angle_min + center_idx * msg.angle_increment
            diff = abs(center_angle - self.cam_angle)
            if diff < best_diff:
                cluster_ranges = ranges[start:end + 1]
                avg_dist = sum(cluster_ranges) / len(cluster_ranges)
                best_diff = diff
                best = (avg_dist, center_angle)

        if best is not None:
            self.buoy_dist, self.buoy_angle = best
        else:
            self.buoy_dist, self.buoy_angle = None, None

    def detect_nearest_buoy(self, msg):
        n = len(msg.ranges)
        ranges = [r if msg.range_min < r < msg.range_max else float('inf') for r in msg.ranges]

        clusters = []
        i = 0
        while i < n:
            if ranges[i] < float('inf'):
                start = i
                while (i + 1 < n and ranges[i + 1] < float('inf')
                       and abs(ranges[i + 1] - ranges[i]) < self.CLUSTER_JUMP_THRESHOLD):
                    i += 1
                clusters.append((start, i))
            i += 1

        best, best_dist = None, float('inf')
        for start, end in clusters:
            cluster_ranges = ranges[start:end + 1]
            avg_dist = sum(cluster_ranges) / len(cluster_ranges)
            if avg_dist < best_dist:
                center_idx = (start + end) // 2
                center_angle = msg.angle_min + center_idx * msg.angle_increment
                best_dist = avg_dist
                best = (avg_dist, center_angle)

        if best is not None:
            self.buoy_dist, self.buoy_angle = best
        else:
            self.buoy_dist, self.buoy_angle = None, None

    def control_loop(self):
        if not self.active:
            return
        if self.phase == 'MOVING':
            self.run_moving()
        elif self.phase == 'APPROACH':
            self.run_approach()
        elif self.phase == 'CIRCLE':
            self.run_circle()
        elif self.phase == 'EXIT':
            self.run_exit()

    def run_moving(self):
        if self.current_lat is None or self.current_lon is None or self.current_heading is None:
            return
        target = MISSION_TARGETS.get('m4s')
        if target is None:
            self.get_logger().warn('m4s 좌표 없음 (field_config.py 확인)', throttle_duration_sec=5.0)
            return
        target_lat, target_lon = target

        distance = self.distance_m(self.current_lat, self.current_lon, target_lat, target_lon)
        if distance <= self.ARRIVAL_RADIUS:
            self.get_logger().info('m4s 도착 → SEARCH 시작 (목표색 확정 대기)')
            self.phase = 'SEARCH'
            return

        self.drive_toward_gps(target_lat, target_lon)

    def run_approach(self):
        if self.buoy_dist is None or self.buoy_angle is None:
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()
        cmd.linear.x = self.CRUISE_SPEED
        cmd.angular.z = self.angle_to_angular(self.buoy_angle)
        self.cmd_pub.publish(cmd)

        if self.buoy_dist <= self.CIRCLE_START_DIST:
            self.get_logger().info('목표 부표 근접(라이다 확인) → 원선회 시작')
            self.phase = 'CIRCLE'
            self.accumulated_turn_deg = 0.0
            self.last_heading = None

    def run_circle(self):
        if self.buoy_dist is None or self.buoy_angle is None:
            self.cmd_pub.publish(Twist())
            return

        if abs(self.accumulated_turn_deg) >= 360.0:
            self.get_logger().info('360도 선회 완료 → EXIT 시작 (m4e로 이동)')
            self.cmd_pub.publish(Twist())
            self.phase = 'EXIT'
            return

        dist_error = self.buoy_dist - self.ORBIT_RADIUS

        cmd = Twist()
        cmd.linear.x = 0.3 + max(-0.1, min(0.1, self.KP_ORBIT * dist_error))
        cmd.angular.z = 0.4 if self.turn_cw else -0.4
        self.cmd_pub.publish(cmd)

    def run_exit(self):
        if self.current_lat is None or self.current_lon is None or self.current_heading is None:
            return
        target = MISSION_TARGETS.get('m4e')
        if target is None:
            self.get_logger().warn('m4e 좌표 없음 (field_config.py 확인)', throttle_duration_sec=5.0)
            return
        target_lat, target_lon = target

        distance = self.distance_m(self.current_lat, self.current_lon, target_lat, target_lon)
        if distance <= self.FINISH_RADIUS:
            self.get_logger().info('m4e 도달! 미션4 완료')
            self.cmd_pub.publish(Twist())
            done = String()
            done.data = self.MY_MISSION
            self.done_pub.publish(done)
            return

        self.drive_toward_gps(target_lat, target_lon)

    def drive_toward_gps(self, target_lat, target_lon):
        bearing = self.bearing_deg(self.current_lat, self.current_lon, target_lat, target_lon)
        angle_error_deg = self.normalize_angle_deg(bearing - self.current_heading)

        cmd = Twist()
        cmd.linear.x = self.CRUISE_SPEED
        cmd.angular.z = self.angle_to_angular(math.radians(angle_error_deg))
        self.cmd_pub.publish(cmd)

    def angle_to_angular(self, target_angle_rad):
        return max(-1.0, min(1.0, self.K_ANGLE * target_angle_rad))

    def normalize_angle_deg(self, angle_deg):
        while angle_deg > 180.0:
            angle_deg -= 360.0
        while angle_deg < -180.0:
            angle_deg += 360.0
        return angle_deg

    @staticmethod
    def bearing_deg(lat1, lon1, lat2, lon2):
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dlambda = math.radians(lon2 - lon1)
        y = math.sin(dlambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
        return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    @staticmethod
    def distance_m(lat1, lon1, lat2, lon2):
        R = 6371000.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def main(args=None):
    rclpy.init(args=args)
    node = Mission4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
