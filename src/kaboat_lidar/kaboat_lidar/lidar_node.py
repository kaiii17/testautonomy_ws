import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info('라이다 노드 시작 - /scan 토픽 구독 중...')

    def scan_callback(self, msg):
        total_points = len(msg.ranges)
        angle_min_deg = msg.angle_min * 180.0 / 3.14159
        angle_max_deg = msg.angle_max * 180.0 / 3.14159

        front_index = total_points // 2
        front_dist = msg.ranges[front_index]

        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid:
            min_dist = min(valid)
            max_dist = max(valid)
        else:
            min_dist = max_dist = 0.0

        self.get_logger().info(
            f'포인트 수: {total_points} | '
            f'각도범위: {angle_min_deg:.0f}~{angle_max_deg:.0f}도 | '
            f'정면거리: {front_dist:.2f}m | '
            f'최소: {min_dist:.2f}m 최대: {max_dist:.2f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
