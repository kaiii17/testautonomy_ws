#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class BoatMarkerNode(Node):
    """
    라이다 좌표 원점을 기준으로 배 형상을 표시한다.

    배 크기:
        길이 0.89 m
        폭   0.42 m

    배 방향:
        선수 = -X
        선미 = +X

    라이다 위치:
        선수 끝에서 선미 방향으로 0.10 m

    라이다 원점 기준:
        선수 끝   x = -0.10 m
        선미 끝   x = +0.79 m
        선체 중심 x = +0.345 m
    """

    LIDAR_FRAME = 'laser'

    BOAT_LENGTH = 0.89
    BOAT_WIDTH = 0.42
    BOAT_HEIGHT = 0.12

    LIDAR_FROM_BOW = 0.10

    BOW_X = -LIDAR_FROM_BOW
    STERN_X = BOAT_LENGTH - LIDAR_FROM_BOW
    BODY_CENTER_X = (BOW_X + STERN_X) / 2.0

    def __init__(self):
        super().__init__('boat_marker_node')

        self.publisher = self.create_publisher(
            MarkerArray,
            '/boat_marker',
            10
        )

        # 배 형상은 고정이므로 1초에 한 번만 발행
        self.timer = self.create_timer(
            1.0,
            self.publish_marker
        )

        self.get_logger().info(
            f'Boat marker started in frame "{self.LIDAR_FRAME}" | '
            f'bow={self.BOW_X:.3f}, '
            f'center={self.BODY_CENTER_X:.3f}, '
            f'stern={self.STERN_X:.3f}'
        )

    def set_common_marker_fields(
        self,
        marker: Marker,
        marker_id: int,
        marker_type: int
    ):
        """
        모든 마커에 공통으로 들어가는 설정.
        """

        marker.header.frame_id = self.LIDAR_FRAME

        # 0초를 사용하면 RViz가 가장 최신 TF를 사용한다.
        # 원격 RViz에서 TF 시간 차이 때문에 깜빡이는 현상을 줄인다.
        marker.header.stamp = rclpy.time.Time().to_msg()

        marker.ns = 'boat'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # TF가 변경될 때 RViz가 마커 위치를 계속 갱신
        marker.frame_locked = True

        # 0초는 삭제하지 않고 계속 유지한다는 의미
        marker.lifetime = Duration(sec=0, nanosec=0)

    def publish_marker(self):
        marker_array = MarkerArray()

        # =====================================================
        # 1. 선체
        # =====================================================
        body = Marker()

        self.set_common_marker_fields(
            marker=body,
            marker_id=0,
            marker_type=Marker.CUBE
        )

        body.pose.position.x = self.BODY_CENTER_X
        body.pose.position.y = 0.0
        body.pose.position.z = self.BOAT_HEIGHT / 2.0

        body.scale.x = self.BOAT_LENGTH
        body.scale.y = self.BOAT_WIDTH
        body.scale.z = self.BOAT_HEIGHT

        body.color.r = 0.10
        body.color.g = 0.45
        body.color.b = 1.00
        body.color.a = 1.00

        marker_array.markers.append(body)

        # =====================================================
        # 2. 선수 삼각형
        # =====================================================
        bow = Marker()

        self.set_common_marker_fields(
            marker=bow,
            marker_id=1,
            marker_type=Marker.TRIANGLE_LIST
        )

        half_width = self.BOAT_WIDTH / 2.0
        marker_z = self.BOAT_HEIGHT + 0.01

        # 선수 꼭짓점
        p1 = Point()
        p1.x = self.BOW_X - 0.10
        p1.y = 0.0
        p1.z = marker_z

        # 선수 삼각형 좌현점
        p2 = Point()
        p2.x = self.BOW_X + 0.12
        p2.y = half_width
        p2.z = marker_z

        # 선수 삼각형 우현점
        p3 = Point()
        p3.x = self.BOW_X + 0.12
        p3.y = -half_width
        p3.z = marker_z

        bow.points = [p1, p2, p3]

        bow.scale.x = 1.0
        bow.scale.y = 1.0
        bow.scale.z = 1.0

        bow.color.r = 0.0
        bow.color.g = 0.0
        bow.color.b = 0.0
        bow.color.a = 1.0

        marker_array.markers.append(bow)

        # =====================================================
        # 3. 선수 방향 화살표
        # =====================================================
        heading = Marker()

        self.set_common_marker_fields(
            marker=heading,
            marker_id=2,
            marker_type=Marker.ARROW
        )

        heading.pose.position.x = self.BODY_CENTER_X
        heading.pose.position.y = 0.0
        heading.pose.position.z = 0.24

        # ARROW의 기본 방향은 +X.
        # Z축 기준 180도 회전해서 선수 방향인 -X로 표시한다.
        heading.pose.orientation.x = 0.0
        heading.pose.orientation.y = 0.0
        heading.pose.orientation.z = 1.0
        heading.pose.orientation.w = 0.0

        heading.scale.x = 0.55
        heading.scale.y = 0.06
        heading.scale.z = 0.10

        heading.color.r = 1.0
        heading.color.g = 0.10
        heading.color.b = 0.10
        heading.color.a = 1.0

        marker_array.markers.append(heading)

        # =====================================================
        # 4. 라이다 원점
        # =====================================================
        lidar_origin = Marker()

        self.set_common_marker_fields(
            marker=lidar_origin,
            marker_id=3,
            marker_type=Marker.SPHERE
        )

        lidar_origin.pose.position.x = 0.0
        lidar_origin.pose.position.y = 0.0
        lidar_origin.pose.position.z = 0.15

        lidar_origin.scale.x = 0.09
        lidar_origin.scale.y = 0.09
        lidar_origin.scale.z = 0.06

        lidar_origin.color.r = 0.0
        lidar_origin.color.g = 1.0
        lidar_origin.color.b = 0.0
        lidar_origin.color.a = 1.0

        marker_array.markers.append(lidar_origin)

        self.publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    node = BoatMarkerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()