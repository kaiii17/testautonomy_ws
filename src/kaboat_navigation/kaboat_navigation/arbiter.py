import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Arbiter(Node):
    """
    조정자 = 최종 스러스터 값 결정.
    - 미션 로직이 원하는 값(cmd_mission)과
      회피 로직이 원하는 값(cmd_avoid)을 각각 받음
    - 회피가 활성(위험 감지)이면 회피 값 우선, 아니면 미션 값
    - 최종 결과를 cmd_final로 발행 → 스러스터 출력 노드가 받음
    """

    def __init__(self):
        super().__init__('arbiter')

        self.mission_cmd = Twist()   # 미션이 원하는 값
        self.avoid_cmd = Twist()     # 회피가 원하는 값
        self.avoid_active = False    # 회피가 지금 개입해야 하는지

        self.create_subscription(Twist, 'cmd_mission', self.mission_cb, 10)
        self.create_subscription(Twist, 'cmd_avoid', self.avoid_cb, 10)
        # 회피 로직이 "지금 위험하다"를 알리는 신호
        self.create_subscription(String, 'avoid/active', self.active_cb, 10)

        self.final_pub = self.create_publisher(Twist, 'cmd_final', 10)

        # 20Hz로 최종 명령 계산
        self.timer = self.create_timer(0.05, self.decide)

        self.get_logger().info('조정자(arbiter) 시작 - 회피 우선 중재')

    def mission_cb(self, msg):
        self.mission_cmd = msg

    def avoid_cb(self, msg):
        self.avoid_cmd = msg

    def active_cb(self, msg):
        # 'danger'면 회피 개입, 'clear'면 미션 값 사용
        self.avoid_active = (msg.data == 'danger')

    def decide(self):
        # 우선순위 규칙: 회피가 급하면 회피, 아니면 미션
        if self.avoid_active:
            self.final_pub.publish(self.avoid_cmd)
        else:
            self.final_pub.publish(self.mission_cmd)


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
