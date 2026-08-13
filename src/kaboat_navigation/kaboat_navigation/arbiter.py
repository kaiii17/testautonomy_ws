import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Arbiter(Node):
    """
    조정자 = 최종 스러스터 값 결정.
    회피 danger + cmd_avoid 신선 -> 회피값
    그 외 cmd_mission 신선 -> 미션값
    둘 다 stale -> 마지막 유효 명령 유지 (완전정지 없음)
    """
    STALE_TIMEOUT_SEC = 1.0

    def __init__(self):
        super().__init__('arbiter')
        self.mission_cmd = Twist()
        self.avoid_cmd = Twist()
        self.avoid_active = False
        self.last_mission_time = None
        self.last_avoid_time = None
        self.last_final_cmd = Twist()

        self.create_subscription(Twist, 'cmd_mission', self.mission_cb, 10)
        self.create_subscription(Twist, 'cmd_avoid', self.avoid_cb, 10)
        self.create_subscription(String, 'avoid/active', self.active_cb, 10)
        self.final_pub = self.create_publisher(Twist, 'cmd_final', 10)
        self.timer = self.create_timer(0.05, self.decide)
        self.get_logger().info('조정자(arbiter) 시작')

    def mission_cb(self, msg):
        self.mission_cmd = msg
        self.last_mission_time = self.get_clock().now()

    def avoid_cb(self, msg):
        self.avoid_cmd = msg
        self.last_avoid_time = self.get_clock().now()

    def active_cb(self, msg):
        self.avoid_active = (msg.data == 'danger')

    def is_stale(self, last_time):
        if last_time is None:
            return True
        elapsed = (self.get_clock().now() - last_time).nanoseconds / 1e9
        return elapsed > self.STALE_TIMEOUT_SEC

    def decide(self):
        if self.avoid_active and not self.is_stale(self.last_avoid_time):
            self.publish_final(self.avoid_cmd)
            return

        if not self.is_stale(self.last_mission_time):
            self.publish_final(self.mission_cmd)
            return

        self.final_pub.publish(self.last_final_cmd)

    def publish_final(self, cmd):
        self.last_final_cmd = cmd
        self.final_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Arbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
