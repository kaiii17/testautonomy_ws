import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestMissionManager(Node):
    """
    단일 미션 테스트용 매니저 - 정식 mission_manager를 대신해서 지정한
    미션 하나만 계속 active/started로 유지한다.

    정식 mission_manager와 다르게 mission/done을 받아도 다음 미션으로
    안 넘어가고, 그 미션이 완료됐다는 로그만 남긴다 (같은 미션을 반복
    테스트하기 위함 - 재시작하려면 mission/started를 다시 보내거나 이
    노드 자체를 재실행).

    사용법:
      ros2 launch kaboat_bringup kaboat_test_mission.launch.py mission:=mission_3

    주의: 정식 mission_manager와 동시에 실행하면 mission/active가
    충돌(둘 다 계속 발행)하므로 절대 같이 켜지 말 것.
    """

    def __init__(self):
        super().__init__('test_mission_manager')

        self.declare_parameter('mission', 'mission_3')
        self.target_mission = self.get_parameter('mission').get_parameter_value().string_value

        self.active_pub = self.create_publisher(String, 'mission/active', 10)
        self.started_pub = self.create_publisher(String, 'mission/started', 10)
        self.create_subscription(String, 'mission/done', self.done_cb, 10)

        self.timer = self.create_timer(0.5, self.publish_active)

        self.get_logger().info(f'★ 단일 미션 테스트 모드 - 대상: {self.target_mission} ★')
        self.publish_started()

    def publish_active(self):
        msg = String()
        msg.data = self.target_mission
        self.active_pub.publish(msg)

    def publish_started(self):
        msg = String()
        msg.data = self.target_mission
        self.started_pub.publish(msg)

    def done_cb(self, msg):
        if msg.data == self.target_mission:
            self.get_logger().info(
                f'{self.target_mission} 완료! (테스트 모드 - 다음 미션 전환 없음, '
                f'재시작하려면 이 노드를 재실행하세요)')


def main(args=None):
    rclpy.init(args=args)
    node = TestMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
