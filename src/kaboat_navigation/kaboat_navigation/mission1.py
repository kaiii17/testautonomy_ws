import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

from kaboat_navigation.field_config import MISSION_TARGETS, TRANSIT_ARRIVAL_RADIUS_M


class Mission1(Node):
    """
    미션 1 - 장애물회피 통과.
    대회 규정: 부표 사이(장애물 구간)를 통과하며 목적지까지 이동. 충돌 시 패널티.

    흐름:
      MOVING : m1s로 GPS 이동 (비례제어 + EMA 스무딩 조향)
      TASK   : m1e 방향으로 동일하게 조향, 도착시 done 발행

    실제 라이다 회피는 이 노드가 신경쓰지 않음 - avoidance.py + arbiter가 전담.
    이 노드는 clear 상태일 때 목표방향으로 부드럽게 조향하는 역할만 맡는다.
    """

    MY_MISSION = 'mission_1'

    HEADING_KP = 1.2
    MAX_ANGULAR = 0.5
    ANGULAR_SMOOTH_ALPHA = 0.15
    TURN_SPEED_MIN_SCALE = 0.5

    def __init__(self):
        super().__init__('mission_1')
        self.active = False
        self.phase = 'MOVING'

        self.current_lat = None
        self.current_lon = None
        self.current_heading = None

        self.smoothed_angular = 0.0
        self.done_logged = False

        self.create_subscription(String, 'mission/active', self.active_cb, 10)
        self.create_subscription(String, 'mission/started', self.started_cb, 10)
        self.create_subscription(String, 'kaboat/gps_nav', self.gps_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_mission', 10)
        self.heading_pub = self.create_publisher(Float32, 'goal/heading', 10)
        self.done_pub = self.create_publisher(String, 'mission/done', 10)

        self.timer = self.create_timer(0.2, self.control_loop)
        self.get_logger().info('mission_1(장애물회피) 노드 시작 - 부드러운 목표추종, 위험회피는 avoidance 전담')

    def active_cb(self, msg):
        self.active = (msg.data == self.MY_MISSION)

    def started_cb(self, msg):
        if msg.data == self.MY_MISSION:
            self.phase = 'MOVING'
            self.done_logged = False
            self.smoothed_angular = 0.0
            self.get_logger().info('mission_1 시작 - 상태 초기화')

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

    def control_loop(self):
        if not self.active or self.current_lat is None:
            return

        if self.phase == 'MOVING':
            self.run_moving()
        elif self.phase == 'TASK':
            self.run_task()

    def run_moving(self):
        start_point = MISSION_TARGETS.get('m1s')
        if start_point is None:
            self.get_logger().warn('m1s 좌표 없음 (field_config.py 확인)', throttle_duration_sec=5.0)
            return

        dist = self.distance_m(self.current_lat, self.current_lon, *start_point)
        if dist > TRANSIT_ARRIVAL_RADIUS_M:
            brg = self.bearing_deg(self.current_lat, self.current_lon, *start_point)
            self.publish_heading(brg)
            self._publish_cmd(0.2, brg)
            return

        self.get_logger().info('m1s 도착 - m1e 목표로 전환')
        self.phase = 'TASK'

    def run_task(self):
        end_point = MISSION_TARGETS.get('m1e')
        if end_point is None:
            self.get_logger().warn('m1e 좌표 없음 (field_config.py 확인)', throttle_duration_sec=5.0)
            return

        dist = self.distance_m(self.current_lat, self.current_lon, *end_point)
        brg = self.bearing_deg(self.current_lat, self.current_lon, *end_point)
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
        done = String()
        done.data = self.MY_MISSION
        self.done_pub.publish(done)
        self.done_logged = True

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
    node = Mission1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
