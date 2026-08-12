import math
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraNode(Node):
    """
    ZED2i 카메라 - 완전 심플 버전.
    색 마스크 -> 컨투어 -> 꼭짓점 개수로만 모양 판별. 그 외 아무것도 없음.
    빨강이 안 잡히는 문제 원인 파악을 위해 각 색의 마스크를 그대로
    'camera/mask_R', 'camera/mask_G', 'camera/mask_B'로 발행 - rosboard로
    직접 눈으로 확인 가능 (전부 까맣게 나오면 색 자체가 안 잡히는 것,
    흰 덩어리가 있는데 detections에 없으면 그다음 단계 문제).
    """

    # 빨강은 HSV 색상환 양끝(0근처, 180근처)에 걸쳐있어 두 범위를 따로 두고
    # OR로 합쳐 씀. 나머지는 단일 범위.
    COLOR_RANGES = {
        'G': ((35, 80, 60), (85, 255, 255)),
        'B': ((100, 100, 100), (130, 255, 255)),
    }
    RED_RANGES = [
        ((0, 70, 60), (12, 255, 255)),
        ((168, 70, 60), (180, 255, 255)),
    ]

    HORIZONTAL_FOV_DEG = 100.0
    MIN_PIXEL_COUNT = 80

    RGB_TOPIC = '/zed/zed_node/rgb/color/rect/image'
    DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered'

    DEBUG_COLORS_BGR = {'R': (0, 0, 255), 'G': (0, 255, 0), 'B': (255, 0, 0)}

    def __init__(self):
        super().__init__('camera_node')

        self.bridge = CvBridge()
        self.latest_depth = None

        self.create_subscription(
            Image, self.RGB_TOPIC, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.DEPTH_TOPIC, self.depth_cb, qos_profile_sensor_data)

        self.detections_pub = self.create_publisher(String, 'camera/detections', 10)
        self.debug_pub = self.create_publisher(Image, 'camera/debug_image', 10)
        self.mask_pubs = {
            c: self.create_publisher(Image, f'camera/mask_{c}', 10)
            for c in ['R', 'G', 'B']
        }

        self.get_logger().info(
            f'카메라 노드 시작 - 완전심플버전 (등록색={list(self.COLOR_RANGES.keys())})')

    def depth_cb(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            pass

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'이미지 변환 실패: {e}', throttle_duration_sec=5.0)
            return

        h, w, _ = frame.shape
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        debug_frame = frame.copy()
        detections = []

        # 빨강: 두 범위를 OR로 합침
        red_mask1 = cv2.inRange(hsv, np.array(self.RED_RANGES[0][0]), np.array(self.RED_RANGES[0][1]))
        red_mask2 = cv2.inRange(hsv, np.array(self.RED_RANGES[1][0]), np.array(self.RED_RANGES[1][1]))
        all_masks = {'R': cv2.bitwise_or(red_mask1, red_mask2)}
        for color_name, (lower, upper) in self.COLOR_RANGES.items():
            all_masks[color_name] = cv2.inRange(hsv, np.array(lower), np.array(upper))

        kernel = np.ones((3, 3), np.uint8)
        kernel_g = np.ones((5, 5), np.uint8)   # 초록은 경계가 특히 지저분해서 더 세게
        for color_name, mask in all_masks.items():
            # 마스크 다듬기: 경계 노이즈(톱니현상) 줄여서 컨투어를 안정화
            k = kernel_g if color_name == 'G' else kernel
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

            self.publish_mask(color_name, mask)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.MIN_PIXEL_COUNT:
                    continue

                M = cv2.moments(contour)
                if M['m00'] == 0:
                    continue
                cx = M['m10'] / M['m00']
                cy = M['m01'] / M['m00']

                distance = self.get_depth_at(int(cx), int(cy))
                angle = self.pixel_to_angle(cx, w)

                perimeter = cv2.arcLength(contour, True)
                # 작은 물체는 둘레도 작아서 상대오차(0.02*perimeter)만 쓰면
                # 경계노이즈를 잘 못 지움 (실측 확인됨). 최소 절대 픽셀(3px)을
                # 같이 강제해서 작은 물체도 충분히 다듬음.
                # (초록만 v가 잘 안 떨어지는 문제는 아직 미해결 - 다음에 재시도)
                epsilon = max(0.02 * perimeter, 3.0)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                vertices = len(approx)
                compactness = area / (perimeter * perimeter) if perimeter > 0 else 0

                if vertices == 3:
                    shape = 'triangle'
                elif vertices == 4:
                    # v=4는 진짜 사각형이거나, 삼각형이 뭉툭하게 잡힌 경우 둘 다 있음.
                    # compactness(면적/둘레^2)로 재구분 - 실측 기준 삼각형은
                    # ~0.03~0.04, 사각형은 그보다 높게 나옴 (경계값 0.055, 실측+이론 종합).
                    if compactness < 0.055:
                        shape = 'triangle'
                    else:
                        shape = 'square'
                elif vertices in (11, 12, 13):
                    shape = 'cross'    # 십자가는 꼭짓점 12개 근처 (실측 확인됨)
                else:
                    shape = 'circle'   # 나머지(5~10, 14+)는 원

                det = {
                    'color': color_name,
                    'angle': round(angle, 4),
                    'shape': shape,
                    'vertices': vertices,
                    'area': int(area),
                    'compactness': round(compactness, 5),
                }
                if distance is not None:
                    det['distance'] = round(distance, 3)
                detections.append(det)

                color_bgr = self.DEBUG_COLORS_BGR.get(color_name, (255, 255, 255))
                cv2.drawContours(debug_frame, [contour], -1, color_bgr, 2)
                cv2.drawMarker(debug_frame, (int(cx), int(cy)),
                                (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
                label = f'{color_name}:{shape}({vertices}) area={int(area)}'
                cv2.putText(debug_frame, label, (int(cx) - 40, int(cy) - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 1)

        out = String()
        out.data = json.dumps(detections)
        self.detections_pub.publish(out)
        self.publish_debug_image(debug_frame)

    def get_depth_at(self, x, y):
        if self.latest_depth is None:
            return None
        h, w = self.latest_depth.shape[:2]
        if not (0 <= x < w and 0 <= y < h):
            return None
        value = float(self.latest_depth[y, x])
        if math.isnan(value) or math.isinf(value) or value <= 0.0:
            return None
        return value

    def pixel_to_angle(self, centroid_x, image_width):
        half_fov_rad = math.radians(self.HORIZONTAL_FOV_DEG / 2.0)
        normalized = (centroid_x / image_width) - 0.5
        return -normalized * 2.0 * half_fov_rad

    def publish_mask(self, color_name, mask):
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            self.mask_pubs[color_name].publish(mask_msg)
        except Exception:
            pass

    def publish_debug_image(self, frame):
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.debug_pub.publish(debug_msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
