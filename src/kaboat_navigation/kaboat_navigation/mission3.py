import math
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from kaboat_navigation.field_config import MISSION_TARGETS, DOCK_SECTORS, MISSION_TARGETS_CONFIG


class Mission3(Node):
    """
    미션 3 = 도킹.

    전체 흐름:
      MOVING : m3s로 GPS 이동. 도착하면 SEARCH로 전환.
      SEARCH : m3s 도착 지점 GPS를 고정점으로 삼아 밀림 보정하며 대기.
               camera/detections에서 TARGET_COLOR+TARGET_SHAPE를 최근
               CONFIRM_WINDOW개 프레임 중 CONFIRM_COUNT번 이상 잡으면 확정.
               확정된 감지들의 평균각도로 좌/중/우 섹터를 판단하고 APPROACH로.
      APPROACH: 확정된 섹터에 해당하는 DOCK_SECTORS 좌표로 GPS 이동.
      DOCKED : 3초 정지 - 도킹 완료 판정.
      TURNING: 진입 시 확정된 섹터 반대쪽(벽에서 먼 쪽)으로 제자리 회전해서
               이탈 준비. 목표 헤딩은 도킹 진입 시점 헤딩의 180도 반대.
      EXIT   : m3e로 GPS 이동. 도착하면 done 발행.

    목표 색/모양은 field_config.py의 MISSION_TARGETS_CONFIG에서 가져옴.
    """

    TARGET_COLOR = MISSION_TARGETS_CONFIG['mission_3']['color']
    TARGET_SHAPE = MISSION_TARGETS_CONFIG['mission_3']['shape']

    ARRIVAL_RADIUS = 1.0
    DOCK_ARRIVAL_RADIUS = 1.0
    FINISH_RADIUS = 1.0
    DOCKED_TIME = 3.0

    CONFIRM_WINDOW = 10
    CONFIRM_COUNT = 4

    SECTOR_ANGLE_THRESHOLD = 0.2

    KP_DIST = 0.3
    KP_ANGLE = 0.5

    CRUISE_SPEED = 0.3
    K_ANGLE = 0.8

    TURN_SPEED_LINEAR = 0.15
    TURN_SPEED_ANGULAR = 0.5
    TURN_DIRECTION_DEFAULT = 1.0    # 가운데 섹터거나 판단불가시 기본값 (1.0=시계, -1.0=반시계)
    TURN_COMPLETE_TOLERANCE_DEG = 15.0

    def __init__(self):
        super().__init__('mission_3')

        self.active = False
        self.phase = 'MOVING'   # MOVING / SEARCH / APPROACH / DOCKED / TURNING / EXIT

        self.current_lat = None
        self.current_lon = None
        self.current_heading = None

        self.hold_lat = None
        self.hold_lon = None

        self.detection_history = []
        self.confirmed_sector = None

        self.docked_start_time = None
        self.entry_heading = None
        self.turn_direction = self.TURN_DIRECTION_DEFAULT

        self.create_subscription(String, 'mission/active', self.active_cb, 10)
        self.create_subscription(String, 'mission/started', self.started_cb, 10)
        self.create_subscription(String, 'camera/detections', self.detections_cb, 10)
        self.create_subscription(String, 'kaboat/gps_nav', self.gps_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_mission', 10)
        self.done_pub = self.create_publisher(String, 'mission/done', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('미션3(도킹) 노드 대기 중')

    def active_cb(self, msg):
        self.active = (msg.data == 'mission_3')

    def started_cb(self, msg):
        if msg.data == 'mission_3':
            self.get_logger().info('★ 미션3 시작 (MOVING -> m3s로 이동) ★')
            self.phase = 'MOVING'
            self.hold_lat = None
            self.hold_lon = None
            self.detection_history = []
            self.confirmed_sector = None
            self.docked_start_time = None
            self.entry_heading = None
            self.turn_direction = self.TURN_DIRECTION_DEFAULT

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

    def detections_cb(self, msg):
        if not self.active or self.phase != 'SEARCH':
            return
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        match = next(
            (d for d in detections
             if d.get('color') == self.TARGET_COLOR and d.get('shape') == self.TARGET_SHAPE),
            None
        )

        if match is not None:
            self.detection_history.append((True, match['angle']))
        else:
            self.detection_history.append((False, None))

        if len(self.detection_history) > self.CONFIRM_WINDOW:
            self.detection_history.pop(0)

        self.check_confirm()

    def check_confirm(self):
        matched = [angle for is_match, angle in self.detection_history if is_match]
        if len(matched) >= self.CONFIRM_COUNT:
            avg_angle = sum(matched) / len(matched)
            self.confirmed_sector = self.angle_to_sector(avg_angle)
            self.get_logger().info(
                f'목표 확정! ({self.TARGET_COLOR}/{self.TARGET_SHAPE}) '
                f'평균각도={math.degrees(avg_angle):.1f}도 -> 섹터={self.confirmed_sector}')
            self.phase = 'APPROACH'

    def angle_to_sector(self, angle_rad):
        if angle_rad > self.SECTOR_ANGLE_THRESHOLD:
            return 'left'
        elif angle_rad < -self.SECTOR_ANGLE_THRESHOLD:
            return 'right'
        else:
            return 'center'

    def control_loop(self):
        if not self.active:
            return

        if self.phase == 'MOVING':
            self.run_moving()
        elif self.phase == 'SEARCH':
            self.run_search_hold()
        elif self.phase == 'APPROACH':
            self.run_approach()
        elif self.phase == 'DOCKED':
            self.run_docked()
        elif self.phase == 'TURNING':
            self.run_turning()
        elif self.phase == 'EXIT':
            self.run_exit()

    def run_moving(self):
        if self.current_lat is None or self.current_lon is None or self.current_heading is None:
            return
        target = MISSION_TARGETS.get('m3s')
        if target is None:
            self.get_logger().warn('m3s 좌표 없음 (field_config.py 확인)', throttle_duration_sec=5.0)
            return
        target_lat, target_lon = target

        distance = self.distance_m(self.current_lat, self.current_lon, target_lat, target_lon)
        if distance <= self.ARRIVAL_RADIUS:
            self.get_logger().info('m3s 도착 → SEARCH 시작 (목표 색/모양 탐색 + 위치고정)')
            self.hold_lat, self.hold_lon = self.current_lat, self.current_lon
            self.phase = 'SEARCH'
            return

        self.drive_toward_gps(target_lat, target_lon)

    def run_search_hold(self):
        if self.current_lat is None or self.current_lon is None or self.current_heading is None:
            return
        if self.hold_lat is None:
            return

        distance = self.distance_m(self.current_lat, self.current_lon, self.hold_lat, self.hold_lon)
        bearing = self.bearing_deg(self.current_lat, self.current_lon, self.hold_lat, self.hold_lon)
        angle_error_deg = self.normalize_angle_deg(bearing - self.current_heading)

        cmd = Twist()
        cmd.linear.x = max(-0.3, min(0.3, self.KP_DIST * distance))
        cmd.angular.z = max(-0.5, min(0.5, self.KP_ANGLE * math.radians(angle_error_deg)))
        self.cmd_pub.publish(cmd)

    def run_approach(self):
        if self.current_lat is None or self.current_lon is None or self.current_heading is None:
            return
        target = DOCK_SECTORS.get(self.confirmed_sector)
        if target is None:
            self.get_logger().warn(
                f'{self.confirmed_sector} 섹터 좌표 없음 (field_config.py 확인)',
                throttle_duration_sec=5.0)
            return
        target_lat, target_lon = target

        distance = self.distance_m(self.current_lat, self.current_lon, target_lat, target_lon)
        if distance <= self.DOCK_ARRIVAL_RADIUS:
            self.get_logger().info(f'{self.confirmed_sector} 섹터 도착 → 도킹 완료 대기(3초)')
            self.cmd_pub.publish(Twist())
            self.phase = 'DOCKED'
            self.docked_start_time = self.get_clock().now()
            return

        self.drive_toward_gps(target_lat, target_lon)

    def run_docked(self):
        self.cmd_pub.publish(Twist())
        elapsed = (self.get_clock().now() - self.docked_start_time).nanoseconds / 1e9
        if elapsed >= self.DOCKED_TIME:
            self.turn_direction = self.decide_turn_direction()
            self.entry_heading = self.current_heading
            self.get_logger().info(
                f'3초 정지 완료 - 섹터={self.confirmed_sector} → '
                f'회전방향={"시계" if self.turn_direction > 0 else "반시계"}로 이탈 회전 시작')
            self.phase = 'TURNING'

    def decide_turn_direction(self):
        """도킹된 섹터 기준으로 벽에서 먼 쪽으로 회전하도록 방향 결정."""
        if self.confirmed_sector == 'left':
            return 1.0    # 왼쪽 슬롯 -> 시계(오른쪽)로 회전
        if self.confirmed_sector == 'right':
            return -1.0   # 오른쪽 슬롯 -> 반시계(왼쪽)로 회전
        return self.TURN_DIRECTION_DEFAULT

    def run_turning(self):
        if self.current_heading is None or self.entry_heading is None:
            return

        target_heading_deg = (self.entry_heading + 180.0) % 360.0
        diff = self.normalize_angle_deg(target_heading_deg - self.current_heading)

        if abs(diff) <= self.TURN_COMPLETE_TOLERANCE_DEG:
            self.get_logger().info('회전 완료 - 도킹장 이탈 성공, m3e로 이동 시작')
            self.phase = 'EXIT'
            return

        cmd = Twist()
        cmd.linear.x = self.TURN_SPEED_LINEAR
        cmd.angular.z = self.turn_direction * self.TURN_SPEED_ANGULAR
        self.cmd_pub.publish(cmd)

    def run_exit(self):
        if self.current_lat is None or self.current_lon is None or self.current_heading is None:
            return
        target = MISSION_TARGETS.get('m3e')
        if target is None:
            self.get_logger().warn('m3e 좌표 없음 (field_config.py 확인)', throttle_duration_sec=5.0)
            return
        target_lat, target_lon = target

        distance = self.distance_m(self.current_lat, self.current_lon, target_lat, target_lon)
        if distance <= self.FINISH_RADIUS:
            self.get_logger().info('m3e 도달! 미션3 완료')
            self.cmd_pub.publish(Twist())
            done = String()
            done.data = 'mission_3'
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
    node = Mission3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
