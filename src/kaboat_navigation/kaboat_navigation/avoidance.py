import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data


class Avoidance(Node):
    """
    회피 노드 - 전체 미션 내내 백그라운드 상시동작 (대회규정, SPOF).
    최후 안전망이라 최대한 단순하게 유지: LiDAR danger_dist 안쪽 각도를
    막힘 처리 -> 안 막힌 구간(gap) 중 제일 넓은 곳으로 조향. 목표방향
    고려 안 함 (그건 각 미션 노드가 자체적으로 함).

    정책: 절대 완전정지하지 않는다.
      - 뚫린 gap이 있으면: 그쪽으로 정상 주행.
      - 뚫린 gap이 없으면(사방이 danger_dist 안쪽): 전체 스캔 중 거리가
        제일 먼(그나마 제일 덜 막힌) 방향으로 저속 전진.
    """

    DANGER_DIST_DEFAULT = 2.5
    MISSION_CONFIG = {
        'mission_1': 2.5, 'mission_2': 1.5, 'mission_3': 2.2,
        'mission_4': 1.5, 'mission_5': 1.5,
    }
    MIN_GAP_WIDTH_DEG = 12.0
    CRAWL_SPEED = 0.15
    AVOID_SPEED = 0.28

    def __init__(self):
        super().__init__('avoidance')
        self.current_mission = None

        self.create_subscription(String, 'mission/active', self.mission_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_avoid', 10)
        self.active_pub = self.create_publisher(String, 'avoid/active', 10)
        self.get_logger().info('회피 노드 시작 (never-stop 정책)')

    def mission_cb(self, msg):
        self.current_mission = msg.data

    def get_config(self):
        return self.MISSION_CONFIG.get(self.current_mission, self.DANGER_DIST_DEFAULT)

    def scan_cb(self, msg):
        try:
            self._process_scan(msg)
        except Exception as e:
            self.get_logger().error(f'scan_cb 예외 - 저속 직진 유지: {e}', throttle_duration_sec=2.0)
            cmd = Twist()
            cmd.linear.x = self.CRAWL_SPEED
            self.cmd_pub.publish(cmd)
            status = String()
            status.data = 'danger'
            self.active_pub.publish(status)

    def _process_scan(self, msg):
        danger_dist = self.get_config()
        min_dist = float('inf')
        blocked = [False] * len(msg.ranges)
        valid_ranges = [None] * len(msg.ranges)

        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                valid_ranges[i] = r
                if r < min_dist:
                    min_dist = r
                if r < danger_dist:
                    blocked[i] = True

        status = String()
        status.data = 'danger' if min_dist < danger_dist else 'clear'
        self.active_pub.publish(status)
        if status.data != 'danger':
            return

        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]
        best = self.find_widest_gap(blocked, angles)

        if best is None:
            self.handle_no_gap(valid_ranges, angles)
            return

        cmd = Twist()
        cmd.linear.x = self.AVOID_SPEED
        cmd.angular.z = max(-1.0, min(1.0, 0.8 * best['center_angle']))
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'회피: {best["width_deg"]:.0f}도 폭, 중심={math.degrees(best["center_angle"]):.1f}도 '
            f'(mission={self.current_mission})')

    def find_widest_gap(self, blocked, angles):
        best = None
        start_idx = None

        def consider(s, e):
            nonlocal best
            width_deg = math.degrees(angles[e] - angles[s])
            if width_deg < self.MIN_GAP_WIDTH_DEG:
                return
            if best is None or width_deg > best['width_deg']:
                center_idx = (s + e) // 2
                best = {'center_angle': angles[center_idx], 'width_deg': width_deg}

        for i, b in enumerate(blocked):
            if not b and start_idx is None:
                start_idx = i
            elif b and start_idx is not None:
                consider(start_idx, i - 1)
                start_idx = None
        if start_idx is not None:
            consider(start_idx, len(blocked) - 1)

        return best

    def handle_no_gap(self, valid_ranges, angles):
        best_idx = None
        best_dist = -1.0
        for i, r in enumerate(valid_ranges):
            if r is not None and r > best_dist:
                best_dist = r
                best_idx = i

        cmd = Twist()
        if best_idx is None:
            cmd.linear.x = self.CRAWL_SPEED
            cmd.angular.z = 0.0
            self.get_logger().warn('유효 스캔 없음 - 저속 직진 유지')
        else:
            target_angle = angles[best_idx]
            cmd.linear.x = self.CRAWL_SPEED
            cmd.angular.z = max(-1.0, min(1.0, 0.8 * target_angle))
            self.get_logger().warn(
                f'뚫린 gap 없음 - 최원거리({best_dist:.2f}m) 방향으로 저속 전진 '
                f'(중심={math.degrees(target_angle):.1f}도)')
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Avoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
