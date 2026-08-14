import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys

COLOR_NAME_KR = {'R': '빨강', 'G': '초록', 'B': '파랑', 'W': '흰색', 'O': '주황', 'Y': '노랑'}
SHAPE_NAME_KR = {'circle': '원형', 'triangle': '삼각형', 'square': '네모', 'cross': '십자'}
ZONE_KR = {'left': '왼쪽', 'center': '중앙', 'right': '오른쪽'}


class CameraTargetDebug(Node):
    """
    카메라 노드가 켜져있고(target_color/target_shape 파라미터 지정된 상태) 발행하는
    camera/detections를 구독해서, is_target=true인 게 나오면 위치 말하고 종료.
    카메라 노드 자체 재구현 없이 그냥 토픽만 구독하는 방식이라 훨씬 가벼움.
    """

    def __init__(self):
        super().__init__('camera_target_debug')
        self.found = False
        self.create_subscription(String, 'camera/detections', self.cb, 10)
        self.get_logger().info('타겟 대기 중... (camera_node가 target_color/target_shape로 켜져있어야 함)')

    def cb(self, msg):
        if self.found:
            return
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        for det in detections:
            if det.get('is_target'):
                self.found = True
                color = COLOR_NAME_KR.get(det['color'], det['color'])
                shape = SHAPE_NAME_KR.get(det['shape'], det['shape'])
                zone = ZONE_KR.get(det['zone'], det['zone'])
                self.get_logger().info(
                    f"★ 찾음! {color} {shape} -> {zone} (area={det['area']}) ★")
                rclpy.shutdown()
                return


def main(args=None):
    rclpy.init(args=args)
    node = CameraTargetDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
