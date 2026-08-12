import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionManager(Node):
    """
    상태 머신 - 미션 순서 관리자.
    하는 일은 딱 2개:
      1. 지금 어떤 미션이 활성인지 계속 알림 (mission/active, 0.5초마다)
      2. 미션이 막 시작된 순간에 1회 알림 (mission/started - 각 미션 노드가
         자기 상태(phase=MOVING 등) 초기화 트리거로 사용)

    미션0(장소이동)을 맨 앞에 포함 - 출발지에서 미션1 시작 전 준비구역(m0e)
    까지 순수 GPS 이동만 담당. 이후 각 미션은 MOVING(mNs로 이동) -> TASK
    (mNe를 기본축 삼아 임무 수행)를 스스로 관리하고, mission/done을 낼 때만
    여기서 다음 미션으로 넘어간다.

    TODO: 대회 규정 확정되면 MISSIONS 순서 실측/조정.
    """

    MISSIONS = [
        'mission_0',   # 장소이동 (출발지 -> m0e)
        'mission_1',   # 항로추종(게이트)
        'mission_2',   # 위치유지
        'mission_3',   # 도킹
        'mission_4',   # 탐색(선회)
        'mission_5',   # 항로추종(추가/예비)
        'finished',
    ]

    def __init__(self):
        super().__init__('mission_manager')

        self.current_index = 0

        self.active_pub = self.create_publisher(String, 'mission/active', 10)
        self.started_pub = self.create_publisher(String, 'mission/started', 10)
        self.create_subscription(String, 'mission/done', self.done_cb, 10)

        self.timer = self.create_timer(0.5, self.publish_active)

        self.get_logger().info(f'미션 매니저 시작 - 현재: {self.current_mission()}')
        self.publish_started()

    def current_mission(self):
        return self.MISSIONS[self.current_index]

    def publish_active(self):
        if self.current_mission() == 'finished':
            return
        msg = String()
        msg.data = self.current_mission()
        self.active_pub.publish(msg)

    def publish_started(self):
        if self.current_mission() == 'finished':
            return
        msg = String()
        msg.data = self.current_mission()
        self.started_pub.publish(msg)

    def done_cb(self, msg):
        if msg.data != self.current_mission():
            return
        if self.current_mission() == 'finished':
            return
        self.get_logger().info(f'{msg.data} 완료! 다음 미션으로 전환')
        self.current_index += 1
        self.get_logger().info(f'현재 미션: {self.current_mission()}')
        self.publish_started()


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
