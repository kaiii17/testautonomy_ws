import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from rclpy.qos import qos_profile_sensor_data


class Avoidance(Node):
    """
    회피 노드 - gap(연속 뚫린 구간) 찾기 + 목표방향(goal_heading) 가중치 점수.

    흐름:
      1. 라이다 각도별 거리 읽기
      2. 막힘/뚫림 판단 (기준거리는 현재 활성 미션별로 조절)
      3. 연속으로 뚫린 gap들 찾기
      4. 각 gap에 점수 = w1*(목표방향 근접도) + w2*(안전도/뚫린정도)
      5. 활성 미션에 따라 w1, w2 조절
      6. 최고점수 gap 중앙으로 heading 틀기 -> cmd_avoid 발행

    목표방향은 goal_heading 노드가 발행하는 'goal/heading' (degrees) 를 구독.

    정책: 절대 완전정지하지 않는다.
      - 뚫린 gap이 있으면: 위 스코어링으로 최적 gap 선택해서 정상 주행.
      - 뚫린 gap이 없으면(사방이 danger_dist 안쪽): 전체 스캔 중 거리가
        제일 먼(그나마 제일 덜 막힌) 방향으로 저속 전진.
      - 예외 발생 시에도 정지 대신 저속 직진 유지.
    """

    DANGER_DIST_DEFAULT = 1.5

    # TODO: 실측 후 조정. mission_2/4는 subin 튜닝값 이식, 가중치는 미션
    # 성격 기준 추정치 - 실측 후 재조정 필요.
    DANGER_DIST_BY_MISSION = {
        'mission_1': 2.5,   # 부표 사이 주행 - 다소 둔감하게
        'mission_2': 1.5,   # 위치유지 - 부표 근접이 전제된 미션
        'mission_3': 2.2,   # 도킹 - 카메라/라이다가 전담, 안전 우선
        'mission_4': 1.5,   # 탐색/선회 - 부표 근접이 전제된 미션
        'mission_5': 1.5,   # 게이트 항로추종 - 일찍 반응
    }

    WEIGHTS_BY_MISSION = {
        'mission_1': (0.6, 0.4),   # 부표사이 주행 - 방향 다소 우선
        'mission_2': (0.3, 0.7),   # 위치유지 - 근접 상황이라 안전 우선
        'mission_3': (0.3, 0.7),   # 도킹 - 카메라/라이다가 전담, 안전 우선
        'mission_4': (0.4, 0.6),   # 탐색/선회 - 안전 다소 우선
        'mission_5': (0.8, 0.2),   # 게이트 - 방향 우선
    }
    WEIGHTS_DEFAULT = (0.5, 0.5)

    MIN_GAP_WIDTH_DEG = 12.0
    CRAWL_SPEED = 0.15   # gap 없을 때 저속 전진 속도
    AVOID_SPEED = 0.28   # gap 있을 때 회피 주행 속도

    def __init__(self):
        super().__init__('avoidance')

        self.current_active = None
        self.goal_heading = None
        self.current_heading = None

        self.create_subscription(String, 'mission/active', self.active_cb, 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Float32, 'goal/heading', self.goal_heading_cb, 10)
        self.create_subscription(String, 'kaboat/gps_nav', self.gps_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_avoid', 10)
        self.active_pub = self.create_publisher(String, 'avoid/active', 10)

        self.get_logger().info('회피 노드 시작 (gap+가중치 스코어링, never-stop 정책)')

    def active_cb(self, msg):
        self.current_active = msg.data

    def goal_heading_cb(self, msg):
        self.goal_heading = math.radians(msg.data)

    def gps_cb(self, msg):
        try:
            for part in msg.data.split(','):
                if part.startswith('imu_heading='):
                    self.current_heading = math.radians(float(part.split('=')[1]))
        except (ValueError, IndexError):
            pass

    def get_danger_dist(self):
        return self.DANGER_DIST_BY_MISSION.get(
            self.current_active, self.DANGER_DIST_DEFAULT)

    def get_weights(self):
        return self.WEIGHTS_BY_MISSION.get(
            self.current_active, self.WEIGHTS_DEFAULT)

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
        danger_dist = self.get_danger_dist()

        n = len(msg.ranges)
        angles = [msg.angle_min + i * msg.angle_increment for i in range(n)]
        valid_ranges = [None] * n
        ranges = []
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                ranges.append(r)
                valid_ranges[i] = r
            else:
                ranges.append(0.0)

        blocked = [r < danger_dist for r in ranges]

        valid = [r for r in ranges if r > 0.0]
        min_dist = min(valid) if valid else float('inf')

        status = String()
        status.data = 'danger' if min_dist < danger_dist else 'clear'
        self.active_pub.publish(status)

        if status.data != 'danger':
            return

        gaps = self.find_gaps(blocked, angles)
        if not gaps:
            self.handle_no_gap(valid_ranges, angles)
            return

        w1, w2 = self.get_weights()
        best_gap = self.score_gaps(gaps, w1, w2)
        target_angle = best_gap['center_angle']

        cmd = Twist()
        cmd.linear.x = self.AVOID_SPEED
        cmd.angular.z = self.angle_to_angular(target_angle)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'회피: gap중심={math.degrees(target_angle):.1f}도 '
            f'(w1={w1} w2={w2} active={self.current_active})')

    def find_gaps(self, blocked, angles):
        gaps = []
        start_idx = None
        for i, b in enumerate(blocked):
            if not b and start_idx is None:
                start_idx = i
            elif b and start_idx is not None:
                gaps.append((start_idx, i - 1))
                start_idx = None
        if start_idx is not None:
            gaps.append((start_idx, len(blocked) - 1))

        result = []
        for s, e in gaps:
            width_deg = math.degrees(angles[e] - angles[s])
            if width_deg < self.MIN_GAP_WIDTH_DEG:
                continue
            center_idx = (s + e) // 2
            result.append({
                'start_angle': angles[s],
                'end_angle': angles[e],
                'center_angle': angles[center_idx],
                'width_deg': width_deg,
            })
        return result

    def score_gaps(self, gaps, w1, w2):
        best = None
        best_score = -999.0

        for gap in gaps:
            if self.goal_heading is not None and self.current_heading is not None:
                goal_relative = self.normalize_angle(self.goal_heading - self.current_heading)
                angle_diff = abs(self.normalize_angle(goal_relative - gap['center_angle']))
                direction_score = 1.0 - (angle_diff / math.pi)
            else:
                direction_score = 1.0 - (abs(gap['center_angle']) / math.pi)

            safety_score = min(gap['width_deg'] / 90.0, 1.0)

            score = w1 * direction_score + w2 * safety_score

            if score > best_score:
                best_score = score
                best = gap

        return best

    def handle_no_gap(self, valid_ranges, angles):
        """뚫린 gap이 아예 없음 (사방이 danger_dist 안쪽).
        정지 대신 그나마 거리가 제일 먼 방향으로 저속 전진."""
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
            cmd.angular.z = self.angle_to_angular(target_angle)
            self.get_logger().warn(
                f'뚫린 gap 없음 - 최원거리({best_dist:.2f}m) 방향으로 저속 전진 '
                f'(중심={math.degrees(target_angle):.1f}도)')
        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def angle_to_angular(self, target_angle):
        K = 0.8
        return max(-1.0, min(1.0, K * target_angle))


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
