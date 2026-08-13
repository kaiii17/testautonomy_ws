import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

from kaboat_navigation.nav_utils import parse_gps_nav, bearing_deg, distance_m, WaypointLogger
from kaboat_navigation.field_config import MISSION_TARGETS, TRANSIT_ARRIVAL_RADIUS_M


class ObstacleCourse(Node):
    """
    mission_1 - 장애물회피 구간.
    실제 '위험시 최후 회피'는 avoidance.py가 전담. 이 노드는 clear 상태일 때
    목표(m1e) 방향으로 부드럽게 조향하는 역할을 맡음 - 비례제어 +
    지수이동평균(EMA) 스무딩으로 급격한 방향전환 없이 goal_heading을 추종.

    흐름:
      1) m1s까지 부드러운 방위각 추종
      2) 도착 -> m1e 방향으로 부드럽게 조향하며 직진
      3) m1e 도착반경 안에 들어오면 완료 판정, mission/done 발행
    """

    MY_MISSION = 'mission_1'
    STATE_TO_START, STATE_TO_END = range(2)

    HEADING_KP = 1.2
    MAX_ANGULAR = 0.5
    ANGULAR_SMOOTH_ALPHA = 0.15
    TURN_SPEED_MIN_SCALE = 0.5

    def __init__(self):
        super().__init__('mission1')
        self.active = False
        self.state = self.STATE_TO_START

        self.current_lat = None
        self.current_lon = None
        self.current_heading = None

        self.smoothed_angular = 0.0

        self.start_logged = False
        self.done_logged = False

        self.create_subscription(String, 'mission/active', self.active_cb, 10)
        self.create_subscription(String, 'mission/started', self.started_cb, 10)
        self.create_subscription(String, 'kaboat/gps_nav', self.gps_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_mission', 10)
        self.heading_pub = self.create_publisher(Float32, 'goal/heading', 10)
        self.done_pub = self.create_publisher(String, 'mission/done', 10)
        self.wp_logger = WaypointLogger(self, self.MY_MISSION)

        self.timer = self.create_timer(0.2, self.control_loop)
        self.get_logger().info('mission_1(장애물회피) 노드 시작 - 부드러운 목표추종, 위험회피는 avoidance 전담')

    def active_cb(self, msg):
        self.active = (msg.data == self.MY_MISSION)

    def started_cb(self, msg):
        if msg.data == self.MY_MISSION:
            self.state = self.STATE_TO_START
            self.start_logged = False
            self.done_logged = False
            self.smoothed_angular = 0.0
            self.get_logger().info('mission_1 시작 - 상태 초기화')

    def gps_cb(self, msg):
        d = parse_gps_nav(msg.data)
        if 'lat' in d:
            self.current_lat = d['lat']
        if 'lon' in d:
            self.current_lon = d['lon']
        if 'imu_heading' in d:
            self.current_heading = d['imu_heading']

        if self.active and not self.start_logged and self.current_lat is not None:
            self.wp_logger.log('start', self.current_lat, self.current_lon, self.current_heading)
            self.start_logged = True

    def control_loop(self):
        if not self.active or self.current_lat is None:
            return

        if self.state == self.STATE_TO_START:
            self.handle_to_start()
        elif self.state == self.STATE_TO_END:
            self.handle_to_end()

    def handle_to_start(self):
        start_point = MISSION_TARGETS.get('m1s')
        if start_point is None:
            self.get_logger().warn('m1s 좌표 없음 (field_config.py 확인)', throttle_duration_sec=5.0)
            return

        dist = distance_m(self.current_lat, self.current_lon, *start_point)
        if dist > TRANSIT_ARRIVAL_RADIUS_M:
            brg = bearing_deg(self.current_lat, self.current_lon, *start_point)
            self.publish_heading(brg)
            self._publish_cmd(0.2, brg)
            return

        self.get_logger().info('m1s 도착 - m1e 목표로 전환')
        self.state = self.STATE_TO_END

    def handle_to_end(self):
        end_point = MISSION_TARGETS.get('m1e')
        if end_point is None:
            self.get_logger().warn('m1e 좌표 없음 (field_config.py 확인)', throttle_duration_sec=5.0)
            return

        dist = distance_m(self.current_lat, self.current_lon, *end_point)
        brg = bearing_deg(self.current_lat, self.current_lon, *end_point)
        self.publish_heading(brg)
        self._publish_cmd(0.3, brg)

        if dist <= TRANSIT_ARRIVAL_RADIUS_M and not self.done_logged:
            self.finish()

    def publish_heading(self, deg):
        h_msg = Float32()
        h_msg.data = deg
        self.heading_pub.publish(h_msg)

    def _heading_error_deg(self, target_deg):
        if target_deg is None or self.current_heading is None:
            return None
        err = target_deg - self.current_heading
        while err > 180.0:
            err -= 360.0
        while err < -180.0:
            err += 360.0
        return err

    def _publish_cmd(self, speed, target_deg):
        error_deg = self._heading_error_deg(target_deg)

        if error_deg is None:
            angular_raw = 0.0
        else:
            angular_raw = self.HEADING_KP * math.radians(error_deg)
            angular_raw = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, angular_raw))

        self.smoothed_angular = (
            self.ANGULAR_SMOOTH_ALPHA * angular_raw
            + (1.0 - self.ANGULAR_SMOOTH_ALPHA) * self.smoothed_angular
        )

        if error_deg is not None:
            turn_ratio = min(abs(error_deg) / 90.0, 1.0)
            speed_scale = 1.0 - turn_ratio * (1.0 - self.TURN_SPEED_MIN_SCALE)
        else:
            speed_scale = 1.0

        cmd = Twist()
        cmd.linear.x = speed * speed_scale
        cmd.angular.z = self.smoothed_angular
        self.cmd_pub.publish(cmd)

    def finish(self):
        self.get_logger().info('m1e 도착 - mission_1 완료')
        self.wp_logger.log('end', self.current_lat, self.current_lon, self.current_heading)
        done = String()
        done.data = self.MY_MISSION
        self.done_pub.publish(done)
        self.done_logged = True


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleCourse()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
