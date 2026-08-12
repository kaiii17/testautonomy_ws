import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
import math

class LidarRVizObstacleNode(Node):
    def __init__(self):
        super().__init__('lidar_rviz_obstacle_node')

        pub_qos_profile = QoSProfile(depth=10)
        pub_qos_profile.reliability = ReliabilityPolicy.RELIABLE

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )

        self.pub = self.create_publisher(MarkerArray, '/lidar_markers', pub_qos_profile)

        self.wall_threshold = 1.2  
        self.buoy_threshold = 2.5  
        self.threshold = 3.0       

    def scan_callback(self, msg: LaserScan):
        markers = MarkerArray()

        # [수정] 깜빡임의 주범인 기존 DELETEALL 구문(5줄)을 완전히 제거했습니다.

        for i, r in enumerate(msg.ranges):
            if r < 0.05 or r > self.threshold or math.isinf(r) or math.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'lidar_obstacle'
            m.id = i
            m.action = Marker.ADD

            # [추가] 마커에 0.2초의 수명을 부여합니다. (새 데이터가 안 오면 알아서 소멸)
            m.lifetime.sec = 0
            m.lifetime.nanosec = 200000000  # 200,000,000ns = 0.2초

            if r <= self.wall_threshold:
                m.type = Marker.CUBE
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.scale.x = 0.3
                m.scale.y = 0.3
                m.scale.z = 0.5
            elif r <= self.buoy_threshold:
                m.type = Marker.SPHERE
                m.color.r = 0.0
                m.color.g = 0.0
                m.color.b = 1.0
                m.scale.x = 0.15
                m.scale.y = 0.15
                m.scale.z = 0.15
            else:
                continue

            m.color.a = 1.0
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.05

            markers.markers.append(m)

        self.pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = LidarRVizObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()