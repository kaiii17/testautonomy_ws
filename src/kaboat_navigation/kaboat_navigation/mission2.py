import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

from kaboat_navigation.nav_utils import parse_gps_nav, bearing_deg, distance_m, WaypointLogger, cluster_scan
from kaboat_navigation.field_config import MISSION_TARGETS, TRANSIT_ARRIVAL_RADIUS_M


class StationKeeping(Node):
    """
    mission_2 - 위치유지 (Station Keeping).
    경기규정: 대상부표 5m 이내에서 5초간 위치를 정지 유지. 부표와 충돌 시 패널티.

    LiDAR 단독으로 최근접 클러스터를 부표로 lock (색상 구분 불필요한 미션).
    데드밴드 P제어 + 히스테리시스 홀딩.

    흐름:
      MOVING : m2s로 GPS 이동. 도착하면 TASK로 전환.
      TASK   : LiDAR로 부표 락온 + 위치유지. 완료 후 m2e로 이동.
    """

    TARGET_DIST = 5.0
    DEADBAND = 1.0
    OUTER_HYSTERESIS = 0.5
    HOLD_SECONDS = 5.0
    FORWARD_CONE_DEG = 90.0
    LOCK_SWITCH_RADIUS_DEG = 15.0
    SEARCH_CRAWL_SPEED = 0.15

    MY_MISSION = 'mission_2'
    STATE_MOVING, STATE_TASK = range(2)

    def __init__(self):
        super().__init__('mission2')
        self.active = False
        self.state = self.STATE_MOVING
        self.current_heading = None
        self.current_lat = None
        self.current_lon = None

        self.locked_target = None
        self.holding = False
        self.hold_start_time = None
        self.start_logged = False
        self.exiting = False

        self.create_subscription(String, 'mission/active', self.active_cb, 10)
        self.create_subscription(String, 'mission/started', self.started_cb, 10)
        self.create_subscription(String, 'kaboat/gps_nav', self.gps_cb, 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_mission', 10)
        self.heading_pub = self.create_publisher(Float32, 'goal/heading', 10)
        self.done_pub = self.create_publisher(String, 'mission/done', 10)
        self.wp_logger = WaypointLogger(self, self.MY_MISSION)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('mission2(위치유지) 노드 시작 - 부표 5m 이내 5초 정지유지')

    def active_cb(self, msg):
        self.active = (msg.data == self.MY_MISSION)

    def started_cb(self, msg):
        if msg.data == self.MY_MISSION:
            self.state = self.STATE_MOVING
            self.locked_target = None
            self.holding = False
            self.hold_start_time = None
            self.start_logged = False
            self.exiting = False
            self.get_logger().info('mission_2 시작 - 상태 초기화 (m2s로 이동)')

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

    def scan_cb(self, msg):
        if not self.active or self.exiting or self.state != self.STATE_TASK:
            return

        n = len(msg.ranges)
        angles = [msg.angle_min + i * msg.angle_increment for i in range(n)]
        clusters = cluster_scan(msg.ranges, angles, msg.range_min, msg.range_max, max_range=10.0)

        cone_rad = math.radians(self.FORWARD_CONE_DEG)
        candidates = [c for c in clusters if abs(c['center_angle']) <= cone_rad]
        if not candidates:
            return

        if self.locked_target is not None:
            prev_bearing_rad = math.radians(self.locked_target['bearing'])
            switch_rad = math.radians(self.LOCK_SWITCH_RADIUS_DEG)
            same_target = [
                c for c in candidates
                if abs(c['center_angle'] - prev_bearing_rad) <= switch_rad
            ]
            chosen = min(same_target, key=lambda c: c['min_range']) if same_target \
                else min(candidates, key=lambda c: c['min_range'])
        else:
            chosen = min(candidates, key=lambda c: c['min_range'])

        self.locked_target = {
            'range': chosen['min_range'],
            'bearing': math.degrees(chosen['center_angle']),
        }

    def control_loop(self):
        if not self.active:
            return

        if self.state == self.STATE_MOVING:
            self.handle_moving()
            return

        if self.exiting:
            self.handle_exit_transit()
            return

        if self.locked_target is None:
            cmd = Twist()
            cmd.linear.x = self.SEARCH_CRAWL_SPEED
            self.cmd_pub.publish(cmd)
            return

        r = self.locked_target['range']
        bearing_rel = self.locked_target['bearing']

        if self.current_heading is not None:
            h_msg = Float32()
            h_msg.data = (self.current_heading + bearing_rel) % 360.0
            self.heading_pub.publish(h_msg)

        error = r - self.TARGET_DIST
        cmd = Twist()
        if abs(error) <= self.DEADBAND / 2.0:
            cmd.linear.x = 0.0
        else:
            K = 0.15
            cmd.linear.x = max(-0.3, min(0.3, K * error))
        cmd.angular.z = max(-0.5, min(0.5, 0.02 * bearing_rel))
        self.cmd_pub.publish(cmd)

        self.update_hold_timer(r)

    def handle_moving(self):
        if self.current_lat is None:
            return

        start_point = MISSION_TARGETS.get('m2s')
        if start_point is None:
            self.get_logger().warn('m2s 좌표 없음 (field_config.py 확인)', throttle_duration_sec=5.0)
            return

        dist = distance_m(self.current_lat, self.current_lon, *start_point)
        if dist > TRANSIT_ARRIVAL_RADIUS_M:
            brg = bearing_deg(self.current_lat, self.current_lon, *start_point)
            h_msg = Float32()
            h_msg.data = brg
            self.heading_pub.publish(h_msg)
            cmd = Twist()
            cmd.linear.x = 0.2
            self.cmd_pub.publish(cmd)
            return

        self.get_logger().info('m2s 도착 - 위치유지 TASK로 전환')
        self.state = self.STATE_TASK

    def update_hold_timer(self, r):
        outer_limit = self.TARGET_DIST + self.OUTER_HYSTERESIS
        inner_ok = r <= self.TARGET_DIST

        if not self.holding:
            if inner_ok:
                self.holding = True
                self.hold_start_time = self.get_clock().now()
                self.get_logger().info('5m 이내 진입 - 5초 유지 타이머 시작')
            return

        if r > outer_limit:
            self.holding = False
            self.hold_start_time = None
            self.get_logger().warn(f'{outer_limit}m 밖으로 이탈 - 타이머 리셋')
            return

        elapsed = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9
        if elapsed >= self.HOLD_SECONDS:
            self.get_logger().info('★★★ mission_2 5초 위치유지 성공 ★★★')
            self.holding = False
            end_point = MISSION_TARGETS.get('m2e')
            if end_point is not None:
                self.exiting = True
                self.get_logger().info('m2e로 이동 시작')
            else:
                self.finish()

    def handle_exit_transit(self):
        point = MISSION_TARGETS.get('m2e')
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

        self.finish()

    def finish(self):
        self.get_logger().info('★★★ mission_2 전체 완료 ★★★')
        self.wp_logger.log('end', self.current_lat, self.current_lon, self.current_heading)
        done = String()
        done.data = self.MY_MISSION
        self.done_pub.publish(done)


def main(args=None):
    rclpy.init(args=args)
    node = StationKeeping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
