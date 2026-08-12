#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformException, TransformListener


class GapPathVisualizer(Node):
    """
    Gap 기반 곡선 경로 시각화 노드.

    입력:
        /map
        /scan
        /clicked_point
        TF: map -> laser

    출력:
        /planned_path
        /planner_markers

    특징:
        1. 저장 지도와 실시간 LaserScan을 모두 검사
        2. Gap 후보마다 보트 운동학 기반 곡선 경로 생성
        3. 정적·실시간 장애물 충돌검사를 모두 통과한 경로만 선택
        4. 새로운 장애물이 나타나면 기존 경로를 자동 재계산
    """

    def __init__(self):
        super().__init__('gap_path_visualizer')

        # ============================================================
        # 1. 프레임과 토픽
        # ============================================================

        self.map_frame = 'map'
        self.laser_frame = 'laser'

        self.map_topic = '/map'
        self.scan_topic = '/scan'
        self.goal_topic = '/clicked_point'

        self.path_topic = '/planned_path'
        self.marker_topic = '/planner_markers'

        # ============================================================
        # 2. 보트 크기 및 안전거리
        # ============================================================

        # 실제 선체 폭 약 0.42 m를 고려한 반폭
        self.boat_half_width = 0.21

        # 장애물과 추가로 떨어질 거리
        self.safety_margin = 0.18

        # 충돌검사에 사용하는 전체 안전 반경
        self.collision_radius = (
            self.boat_half_width + self.safety_margin
        )

        # 지도에서 이 값 이상이면 장애물로 판단
        self.occupied_threshold = 50

        # 미확인 영역(-1)을 장애물로 볼지 여부
        self.unknown_is_blocked = False

        # ============================================================
        # 3. Gap 탐색 설정
        # ============================================================

        # 라이다 정면 탐색 범위
        self.front_angle_min = math.radians(-80.0)
        self.front_angle_max = math.radians(80.0)

        # 이 거리보다 가까운 측정값은 장애물
        self.obstacle_distance = 2.5

        # Gap으로 인정할 최소 각도 폭
        self.minimum_gap_angle = math.radians(12.0)

        # 후보 Gap 최대 개수
        self.maximum_gap_candidates = 15

        # Gap 방향에 가상 중간 목표점을 놓을 거리
        self.gap_lookahead_distance = 2.0

        # ============================================================
        # 4. 곡선 경로 예측 설정
        # ============================================================

        # 실제 주행 알고리즘의 선속과 유사하게 설정
        self.linear_speed = 0.25

        # heading 오차에 대한 angular.z 비례계수
        self.heading_kp = 1.4

        # 최대 각속도
        self.maximum_angular_speed = 0.65

        # 미래 경로 예측 시간
        self.prediction_time = 6.0

        # 시뮬레이션 시간 간격
        self.prediction_dt = 0.10

        # Gap 중간점을 지나 목표점으로 전환하는 거리
        self.gap_reached_distance = 0.55

        # 목표 도착 판정 거리
        self.goal_reached_distance = 0.40

        # 매 몇 번째 점마다 충돌검사할지
        self.collision_check_stride = 1

        # ============================================================
        # 5. 경로 점수 가중치
        # ============================================================

        self.weight_goal_direction = 0.25
        self.weight_gap_width = 0.20
        self.weight_goal_progress = 0.30
        self.weight_clearance = 0.15
        self.weight_steering = 0.10

        # ============================================================
        # 6. 내부 데이터
        # ============================================================

        self.map_msg: Optional[OccupancyGrid] = None
        self.scan_msg: Optional[LaserScan] = None
        self.goal_point: Optional[PointStamped] = None

        self.last_result_text = 'WAITING FOR GOAL'

        # ============================================================
        # 7. TF
        # ============================================================

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # ============================================================
        # 8. QoS
        # ============================================================

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ============================================================
        # 9. 구독
        # ============================================================

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.goal_sub = self.create_subscription(
            PointStamped,
            self.goal_topic,
            self.goal_callback,
            10,
        )

        # ============================================================
        # 10. 발행
        # ============================================================

        self.path_pub = self.create_publisher(
            Path,
            self.path_topic,
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10,
        )

        # 5 Hz로 반복 계획
        self.timer = self.create_timer(
            0.20,
            self.plan_callback,
        )

        self.get_logger().info(
            'Gap path visualizer started'
        )
        self.get_logger().info(
            f'collision radius: {self.collision_radius:.2f} m'
        )

    # ================================================================
    # 콜백
    # ================================================================

    def map_callback(self, msg: OccupancyGrid):
        first_map = self.map_msg is None
        self.map_msg = msg

        if first_map:
            self.get_logger().info(
                f'Map received: '
                f'{msg.info.width} x {msg.info.height}, '
                f'resolution={msg.info.resolution:.3f}'
            )

    def scan_callback(self, msg: LaserScan):
        self.scan_msg = msg

        # 새로운 스캔이 들어올 때마다 timer에서 경로가 다시 계산된다.
        # 따라서 새 장애물이 나타나면 기존 경로가 자동 폐기될 수 있다.

    def goal_callback(self, msg: PointStamped):
        goal = PointStamped()
        goal.header = msg.header
        goal.point = msg.point

        # Publish Point가 map 프레임이라는 전제
        if not goal.header.frame_id:
            goal.header.frame_id = self.map_frame

        self.goal_point = goal

        self.get_logger().info(
            f'Goal received: '
            f'x={goal.point.x:.2f}, '
            f'y={goal.point.y:.2f}'
        )

    # ================================================================
    # 메인 계획 함수
    # ================================================================

    def plan_callback(self):
        if self.goal_point is None:
            return

        if self.map_msg is None:
            self.publish_empty_path()
            self.publish_status_only(
                'WAITING FOR MAP'
            )
            return

        if self.scan_msg is None:
            self.publish_empty_path()
            self.publish_status_only(
                'WAITING FOR SCAN'
            )
            return

        boat_pose = self.get_boat_pose_in_map()

        if boat_pose is None:
            self.publish_empty_path()
            self.publish_status_only(
                'TF NOT AVAILABLE'
            )
            return

        boat_x, boat_y, boat_yaw = boat_pose

        goal_x = self.goal_point.point.x
        goal_y = self.goal_point.point.y

        goal_distance = math.hypot(
            goal_x - boat_x,
            goal_y - boat_y,
        )

        if goal_distance <= self.goal_reached_distance:
            self.publish_empty_path()
            self.publish_visualization(
                boat_pose=boat_pose,
                gap_candidates=[],
                candidate_trajectories=[],
                selected_candidate=None,
                result_text='GOAL REACHED',
            )
            return

        gap_candidates = self.detect_gaps()

        # 목표 방향이 현재 열린 공간이면 직접 목표 방향 후보도 추가
        direct_candidate = self.make_direct_goal_candidate(
            boat_x,
            boat_y,
            boat_yaw,
            goal_x,
            goal_y,
        )

        if direct_candidate is not None:
            gap_candidates.append(direct_candidate)

        gap_candidates = self.remove_duplicate_candidates(
            gap_candidates
        )

        candidate_results = []

        for candidate in gap_candidates:
            trajectory = self.generate_curved_trajectory(
                boat_x=boat_x,
                boat_y=boat_y,
                boat_yaw=boat_yaw,
                goal_x=goal_x,
                goal_y=goal_y,
                gap_angle=candidate['angle'],
            )

            if len(trajectory) < 2:
                continue

            map_safe = self.trajectory_is_safe_in_map(
                trajectory
            )

            scan_safe = self.trajectory_is_safe_in_scan(
                trajectory,
                boat_x,
                boat_y,
                boat_yaw,
            )

            if not map_safe or not scan_safe:
                candidate['map_safe'] = map_safe
                candidate['scan_safe'] = scan_safe
                candidate['valid'] = False
                candidate['score'] = -999.0

                candidate_results.append(
                    {
                        'candidate': candidate,
                        'trajectory': trajectory,
                    }
                )
                continue

            minimum_clearance = self.calculate_scan_clearance(
                trajectory,
                boat_x,
                boat_y,
                boat_yaw,
            )

            score = self.calculate_candidate_score(
                candidate=candidate,
                trajectory=trajectory,
                boat_x=boat_x,
                boat_y=boat_y,
                boat_yaw=boat_yaw,
                goal_x=goal_x,
                goal_y=goal_y,
                minimum_clearance=minimum_clearance,
            )

            candidate['map_safe'] = True
            candidate['scan_safe'] = True
            candidate['valid'] = True
            candidate['score'] = score
            candidate['clearance'] = minimum_clearance

            candidate_results.append(
                {
                    'candidate': candidate,
                    'trajectory': trajectory,
                }
            )

        valid_results = [
            result
            for result in candidate_results
            if result['candidate'].get('valid', False)
        ]

        if not valid_results:
            self.publish_empty_path()

            self.publish_visualization(
                boat_pose=boat_pose,
                gap_candidates=gap_candidates,
                candidate_trajectories=candidate_results,
                selected_candidate=None,
                result_text=(
                    f'NO SAFE PATH | '
                    f'GAPS={len(gap_candidates)}'
                ),
            )

            return

        selected = max(
            valid_results,
            key=lambda item: item['candidate']['score'],
        )

        self.publish_path(
            selected['trajectory']
        )

        selected_angle_deg = math.degrees(
            selected['candidate']['angle']
        )

        result_text = (
            f'PATH FOUND | '
            f'ANGLE={selected_angle_deg:.1f} deg | '
            f'SCORE={selected["candidate"]["score"]:.2f}'
        )

        self.publish_visualization(
            boat_pose=boat_pose,
            gap_candidates=gap_candidates,
            candidate_trajectories=candidate_results,
            selected_candidate=selected,
            result_text=result_text,
        )

    # ================================================================
    # TF
    # ================================================================

    def get_boat_pose_in_map(
        self,
    ) -> Optional[Tuple[float, float, float]]:

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.laser_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.10),
            )

        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        yaw = self.quaternion_to_yaw(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )

        return (
            translation.x,
            translation.y,
            yaw,
        )

    @staticmethod
    def quaternion_to_yaw(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> float:

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    # ================================================================
    # Gap 탐색
    # ================================================================

    def detect_gaps(self) -> List[dict]:
        scan = self.scan_msg

        if scan is None:
            return []

        samples = []

        for index, measured_range in enumerate(scan.ranges):
            angle = (
                scan.angle_min
                + index * scan.angle_increment
            )

            if (
                angle < self.front_angle_min
                or angle > self.front_angle_max
            ):
                continue

            valid_measurement = (
                math.isfinite(measured_range)
                and scan.range_min <= measured_range <= scan.range_max
            )

            if not valid_measurement:
                free = True
                usable_range = scan.range_max
            else:
                free = measured_range > self.obstacle_distance
                usable_range = measured_range

            samples.append(
                {
                    'index': index,
                    'angle': angle,
                    'range': usable_range,
                    'free': free,
                }
            )

        if not samples:
            return []

        gaps = []
        gap_start = None

        for sample_index, sample in enumerate(samples):
            if sample['free'] and gap_start is None:
                gap_start = sample_index

            gap_finished = (
                not sample['free']
                or sample_index == len(samples) - 1
            )

            if gap_start is not None and gap_finished:
                if sample['free']:
                    gap_end = sample_index
                else:
                    gap_end = sample_index - 1

                if gap_end >= gap_start:
                    start_angle = samples[gap_start]['angle']
                    end_angle = samples[gap_end]['angle']
                    gap_width_angle = end_angle - start_angle

                    if gap_width_angle >= self.minimum_gap_angle:
                        center_index = (
                            gap_start + gap_end
                        ) // 2

                        center_angle = samples[
                            center_index
                        ]['angle']

                        gap_ranges = [
                            samples[i]['range']
                            for i in range(
                                gap_start,
                                gap_end + 1,
                            )
                        ]

                        minimum_range = min(gap_ranges)

                        physical_width = (
                            2.0
                            * minimum_range
                            * math.sin(
                                gap_width_angle / 2.0
                            )
                        )

                        gaps.append(
                            {
                                'angle': center_angle,
                                'start_angle': start_angle,
                                'end_angle': end_angle,
                                'angular_width': gap_width_angle,
                                'physical_width': physical_width,
                                'minimum_range': minimum_range,
                                'source': 'gap',
                            }
                        )

                gap_start = None

        gaps.sort(
            key=lambda gap: gap['physical_width'],
            reverse=True,
        )

        return gaps[:self.maximum_gap_candidates]

    def make_direct_goal_candidate(
        self,
        boat_x: float,
        boat_y: float,
        boat_yaw: float,
        goal_x: float,
        goal_y: float,
    ) -> Optional[dict]:

        global_goal_angle = math.atan2(
            goal_y - boat_y,
            goal_x - boat_x,
        )

        relative_goal_angle = self.normalize_angle(
            global_goal_angle - boat_yaw
        )

        if (
            relative_goal_angle < self.front_angle_min
            or relative_goal_angle > self.front_angle_max
        ):
            return None

        if not self.scan_direction_is_free(
            relative_goal_angle
        ):
            return None

        return {
            'angle': relative_goal_angle,
            'start_angle': relative_goal_angle,
            'end_angle': relative_goal_angle,
            'angular_width': math.radians(10.0),
            'physical_width': 0.8,
            'minimum_range': self.scan_msg.range_max,
            'source': 'direct_goal',
        }

    def scan_direction_is_free(
        self,
        relative_angle: float,
    ) -> bool:

        scan = self.scan_msg

        if scan is None:
            return False

        index = int(
            round(
                (relative_angle - scan.angle_min)
                / scan.angle_increment
            )
        )

        if index < 0 or index >= len(scan.ranges):
            return False

        half_window = max(
            1,
            int(
                math.atan2(
                    self.collision_radius,
                    max(self.gap_lookahead_distance, 0.1),
                )
                / abs(scan.angle_increment)
            ),
        )

        start = max(0, index - half_window)
        end = min(
            len(scan.ranges) - 1,
            index + half_window,
        )

        for beam_index in range(start, end + 1):
            measured_range = scan.ranges[beam_index]

            if not math.isfinite(measured_range):
                continue

            if measured_range < (
                self.gap_lookahead_distance
                + self.collision_radius
            ):
                return False

        return True

    @staticmethod
    def remove_duplicate_candidates(
        candidates: List[dict],
    ) -> List[dict]:

        if not candidates:
            return []

        candidates = sorted(
            candidates,
            key=lambda item: item['angle'],
        )

        filtered = []

        minimum_separation = math.radians(5.0)

        for candidate in candidates:
            if not filtered:
                filtered.append(candidate)
                continue

            previous = filtered[-1]

            if abs(
                candidate['angle']
                - previous['angle']
            ) < minimum_separation:

                if (
                    candidate.get('physical_width', 0.0)
                    > previous.get('physical_width', 0.0)
                ):
                    filtered[-1] = candidate
            else:
                filtered.append(candidate)

        return filtered

    # ================================================================
    # 곡선 경로 생성
    # ================================================================

    def generate_curved_trajectory(
        self,
        boat_x: float,
        boat_y: float,
        boat_yaw: float,
        goal_x: float,
        goal_y: float,
        gap_angle: float,
    ) -> List[Tuple[float, float, float]]:

        trajectory = []

        x = boat_x
        y = boat_y
        yaw = boat_yaw

        # Gap 방향의 가상 중간 목표
        global_gap_heading = self.normalize_angle(
            boat_yaw + gap_angle
        )

        gap_target_x = (
            boat_x
            + self.gap_lookahead_distance
            * math.cos(global_gap_heading)
        )

        gap_target_y = (
            boat_y
            + self.gap_lookahead_distance
            * math.sin(global_gap_heading)
        )

        using_gap_target = True

        number_of_steps = int(
            self.prediction_time
            / self.prediction_dt
        )

        trajectory.append(
            (x, y, yaw)
        )

        for _ in range(number_of_steps):
            if using_gap_target:
                target_x = gap_target_x
                target_y = gap_target_y

                distance_to_gap = math.hypot(
                    target_x - x,
                    target_y - y,
                )

                if distance_to_gap <= self.gap_reached_distance:
                    using_gap_target = False
                    target_x = goal_x
                    target_y = goal_y
            else:
                target_x = goal_x
                target_y = goal_y

            distance_to_goal = math.hypot(
                goal_x - x,
                goal_y - y,
            )

            if distance_to_goal <= self.goal_reached_distance:
                break

            desired_heading = math.atan2(
                target_y - y,
                target_x - x,
            )

            heading_error = self.normalize_angle(
                desired_heading - yaw
            )

            angular_speed = self.clamp(
                self.heading_kp * heading_error,
                -self.maximum_angular_speed,
                self.maximum_angular_speed,
            )

            # 회전이 매우 클 때는 선속 감소
            turn_ratio = min(
                abs(angular_speed)
                / max(self.maximum_angular_speed, 0.01),
                1.0,
            )

            current_linear_speed = (
                self.linear_speed
                * (1.0 - 0.55 * turn_ratio)
            )

            current_linear_speed = max(
                0.08,
                current_linear_speed,
            )

            # Unicycle 운동학
            x += (
                current_linear_speed
                * math.cos(yaw)
                * self.prediction_dt
            )

            y += (
                current_linear_speed
                * math.sin(yaw)
                * self.prediction_dt
            )

            yaw = self.normalize_angle(
                yaw
                + angular_speed
                * self.prediction_dt
            )

            trajectory.append(
                (x, y, yaw)
            )

        return trajectory

    # ================================================================
    # 지도 충돌검사
    # ================================================================

    def trajectory_is_safe_in_map(
        self,
        trajectory: List[Tuple[float, float, float]],
    ) -> bool:

        for index in range(
            0,
            len(trajectory),
            self.collision_check_stride,
        ):
            x, y, _ = trajectory[index]

            if self.point_collides_with_map(
                x,
                y,
                self.collision_radius,
            ):
                return False

        return True

    def point_collides_with_map(
        self,
        world_x: float,
        world_y: float,
        radius: float,
    ) -> bool:

        if self.map_msg is None:
            return True

        resolution = self.map_msg.info.resolution

        sample_step = max(
            resolution,
            radius / 3.0,
        )

        offset_x = -radius

        while offset_x <= radius:
            offset_y = -radius

            while offset_y <= radius:
                if (
                    offset_x * offset_x
                    + offset_y * offset_y
                    <= radius * radius
                ):
                    if self.map_cell_is_blocked(
                        world_x + offset_x,
                        world_y + offset_y,
                    ):
                        return True

                offset_y += sample_step

            offset_x += sample_step

        return False

    def map_cell_is_blocked(
        self,
        world_x: float,
        world_y: float,
    ) -> bool:

        map_msg = self.map_msg

        if map_msg is None:
            return True

        origin = map_msg.info.origin.position
        resolution = map_msg.info.resolution

        map_x = int(
            math.floor(
                (world_x - origin.x)
                / resolution
            )
        )

        map_y = int(
            math.floor(
                (world_y - origin.y)
                / resolution
            )
        )

        if (
            map_x < 0
            or map_y < 0
            or map_x >= map_msg.info.width
            or map_y >= map_msg.info.height
        ):
            return True

        index = (
            map_y * map_msg.info.width
            + map_x
        )

        occupancy = map_msg.data[index]

        if occupancy < 0:
            return self.unknown_is_blocked

        return occupancy >= self.occupied_threshold

    # ================================================================
    # 실시간 LaserScan 충돌검사
    # ================================================================

    def trajectory_is_safe_in_scan(
        self,
        trajectory: List[Tuple[float, float, float]],
        laser_x: float,
        laser_y: float,
        laser_yaw: float,
    ) -> bool:

        scan = self.scan_msg

        if scan is None:
            return False

        # 너무 먼 미래 경로는 현재 라이다 한 장으로 판단하기 어렵다.
        maximum_scan_check_distance = min(
            scan.range_max,
            self.obstacle_distance + 1.0,
        )

        for index in range(
            1,
            len(trajectory),
            self.collision_check_stride,
        ):
            world_x, world_y, _ = trajectory[index]

            local_x, local_y = self.map_to_laser(
                world_x,
                world_y,
                laser_x,
                laser_y,
                laser_yaw,
            )

            point_range = math.hypot(
                local_x,
                local_y,
            )

            if point_range > maximum_scan_check_distance:
                continue

            point_angle = math.atan2(
                local_y,
                local_x,
            )

            if (
                point_angle < scan.angle_min
                or point_angle > scan.angle_max
            ):
                continue

            beam_index = int(
                round(
                    (point_angle - scan.angle_min)
                    / scan.angle_increment
                )
            )

            if (
                beam_index < 0
                or beam_index >= len(scan.ranges)
            ):
                continue

            angular_window = math.atan2(
                self.collision_radius,
                max(point_range, 0.05),
            )

            beam_window = max(
                1,
                int(
                    angular_window
                    / abs(scan.angle_increment)
                ),
            )

            start_index = max(
                0,
                beam_index - beam_window,
            )

            end_index = min(
                len(scan.ranges) - 1,
                beam_index + beam_window,
            )

            for current_beam in range(
                start_index,
                end_index + 1,
            ):
                measured_range = scan.ranges[current_beam]

                if not math.isfinite(measured_range):
                    continue

                if measured_range < scan.range_min:
                    continue

                # 경로점까지의 거리보다 장애물이 앞에 있으면 충돌
                if measured_range <= (
                    point_range
                    + self.collision_radius
                ):
                    return False

        return True

    def calculate_scan_clearance(
        self,
        trajectory: List[Tuple[float, float, float]],
        laser_x: float,
        laser_y: float,
        laser_yaw: float,
    ) -> float:

        scan = self.scan_msg

        if scan is None:
            return 0.0

        minimum_clearance = scan.range_max

        for world_x, world_y, _ in trajectory[1:]:
            local_x, local_y = self.map_to_laser(
                world_x,
                world_y,
                laser_x,
                laser_y,
                laser_yaw,
            )

            point_range = math.hypot(
                local_x,
                local_y,
            )

            point_angle = math.atan2(
                local_y,
                local_x,
            )

            if (
                point_angle < scan.angle_min
                or point_angle > scan.angle_max
            ):
                continue

            beam_index = int(
                round(
                    (point_angle - scan.angle_min)
                    / scan.angle_increment
                )
            )

            if (
                beam_index < 0
                or beam_index >= len(scan.ranges)
            ):
                continue

            measured_range = scan.ranges[beam_index]

            if not math.isfinite(measured_range):
                continue

            clearance = measured_range - point_range

            minimum_clearance = min(
                minimum_clearance,
                clearance,
            )

        return max(
            0.0,
            minimum_clearance,
        )

    @staticmethod
    def map_to_laser(
        world_x: float,
        world_y: float,
        laser_x: float,
        laser_y: float,
        laser_yaw: float,
    ) -> Tuple[float, float]:

        dx = world_x - laser_x
        dy = world_y - laser_y

        cos_yaw = math.cos(laser_yaw)
        sin_yaw = math.sin(laser_yaw)

        local_x = (
            cos_yaw * dx
            + sin_yaw * dy
        )

        local_y = (
            -sin_yaw * dx
            + cos_yaw * dy
        )

        return local_x, local_y

    # ================================================================
    # 점수 계산
    # ================================================================

    def calculate_candidate_score(
        self,
        candidate: dict,
        trajectory: List[Tuple[float, float, float]],
        boat_x: float,
        boat_y: float,
        boat_yaw: float,
        goal_x: float,
        goal_y: float,
        minimum_clearance: float,
    ) -> float:

        initial_goal_distance = math.hypot(
            goal_x - boat_x,
            goal_y - boat_y,
        )

        end_x, end_y, _ = trajectory[-1]

        final_goal_distance = math.hypot(
            goal_x - end_x,
            goal_y - end_y,
        )

        global_goal_heading = math.atan2(
            goal_y - boat_y,
            goal_x - boat_x,
        )

        relative_goal_heading = self.normalize_angle(
            global_goal_heading - boat_yaw
        )

        heading_difference = abs(
            self.normalize_angle(
                candidate['angle']
                - relative_goal_heading
            )
        )

        goal_direction_score = (
            1.0
            - min(
                heading_difference / math.pi,
                1.0,
            )
        )

        physical_width = candidate.get(
            'physical_width',
            0.0,
        )

        gap_width_score = min(
            physical_width / 2.0,
            1.0,
        )

        progress = (
            initial_goal_distance
            - final_goal_distance
        )

        goal_progress_score = self.clamp(
            progress
            / max(
                self.linear_speed
                * self.prediction_time,
                0.1,
            ),
            -1.0,
            1.0,
        )

        clearance_score = min(
            minimum_clearance / 1.5,
            1.0,
        )

        steering_score = (
            1.0
            - min(
                abs(candidate['angle'])
                / max(
                    abs(self.front_angle_max),
                    0.01,
                ),
                1.0,
            )
        )

        total_score = (
            self.weight_goal_direction
            * goal_direction_score
            + self.weight_gap_width
            * gap_width_score
            + self.weight_goal_progress
            * goal_progress_score
            + self.weight_clearance
            * clearance_score
            + self.weight_steering
            * steering_score
        )

        return total_score

    # ================================================================
    # Path 발행
    # ================================================================

    def publish_path(
        self,
        trajectory: List[Tuple[float, float, float]],
    ):

        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y, yaw in trajectory:
            pose = PoseStamped()

            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.05

            qx, qy, qz, qw = self.yaw_to_quaternion(
                yaw
            )

            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path.poses.append(pose)

        self.path_pub.publish(path)

    def publish_empty_path(self):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        self.path_pub.publish(path)

    # ================================================================
    # RViz Marker
    # ================================================================

    def publish_visualization(
        self,
        boat_pose: Tuple[float, float, float],
        gap_candidates: List[dict],
        candidate_trajectories: List[dict],
        selected_candidate: Optional[dict],
        result_text: str,
    ):

        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL

        marker_array.markers.append(
            delete_marker
        )

        marker_id = 0
        stamp = self.get_clock().now().to_msg()

        # ------------------------------------------------------------
        # 목표점: 분홍색
        # ------------------------------------------------------------

        if self.goal_point is not None:
            goal_marker = self.make_sphere_marker(
                marker_id=marker_id,
                namespace='goal',
                x=self.goal_point.point.x,
                y=self.goal_point.point.y,
                z=0.20,
                scale=0.28,
                color=self.make_color(
                    1.0,
                    0.0,
                    1.0,
                    1.0,
                ),
                stamp=stamp,
            )

            marker_array.markers.append(
                goal_marker
            )

            marker_id += 1

        boat_x, boat_y, boat_yaw = boat_pose

        # ------------------------------------------------------------
        # Gap 후보점: 노란색
        # ------------------------------------------------------------

        for candidate in gap_candidates:
            global_angle = self.normalize_angle(
                boat_yaw + candidate['angle']
            )

            candidate_x = (
                boat_x
                + self.gap_lookahead_distance
                * math.cos(global_angle)
            )

            candidate_y = (
                boat_y
                + self.gap_lookahead_distance
                * math.sin(global_angle)
            )

            candidate_marker = self.make_sphere_marker(
                marker_id=marker_id,
                namespace='gap_candidates',
                x=candidate_x,
                y=candidate_y,
                z=0.12,
                scale=0.16,
                color=self.make_color(
                    1.0,
                    0.85,
                    0.0,
                    0.9,
                ),
                stamp=stamp,
            )

            marker_array.markers.append(
                candidate_marker
            )

            marker_id += 1

        # ------------------------------------------------------------
        # 모든 곡선 후보
        # 유효 경로: 연한 파랑
        # 충돌 경로: 빨강
        # ------------------------------------------------------------

        for result in candidate_trajectories:
            candidate = result['candidate']
            trajectory = result['trajectory']

            if candidate.get('valid', False):
                color = self.make_color(
                    0.15,
                    0.65,
                    1.0,
                    0.35,
                )
            else:
                color = self.make_color(
                    1.0,
                    0.15,
                    0.15,
                    0.20,
                )

            line_marker = self.make_line_marker(
                marker_id=marker_id,
                namespace='candidate_paths',
                trajectory=trajectory,
                width=0.025,
                color=color,
                stamp=stamp,
            )

            marker_array.markers.append(
                line_marker
            )

            marker_id += 1

        # ------------------------------------------------------------
        # 선택 경로 및 선택 Gap: 초록색
        # ------------------------------------------------------------

        if selected_candidate is not None:
            selected_data = selected_candidate['candidate']
            selected_trajectory = selected_candidate['trajectory']

            selected_line = self.make_line_marker(
                marker_id=marker_id,
                namespace='selected_path',
                trajectory=selected_trajectory,
                width=0.08,
                color=self.make_color(
                    0.0,
                    1.0,
                    0.15,
                    1.0,
                ),
                stamp=stamp,
            )

            marker_array.markers.append(
                selected_line
            )

            marker_id += 1

            global_selected_angle = self.normalize_angle(
                boat_yaw
                + selected_data['angle']
            )

            selected_x = (
                boat_x
                + self.gap_lookahead_distance
                * math.cos(global_selected_angle)
            )

            selected_y = (
                boat_y
                + self.gap_lookahead_distance
                * math.sin(global_selected_angle)
            )

            selected_marker = self.make_sphere_marker(
                marker_id=marker_id,
                namespace='selected_gap',
                x=selected_x,
                y=selected_y,
                z=0.18,
                scale=0.24,
                color=self.make_color(
                    0.0,
                    1.0,
                    0.15,
                    1.0,
                ),
                stamp=stamp,
            )

            marker_array.markers.append(
                selected_marker
            )

            marker_id += 1

        # ------------------------------------------------------------
        # 상태 텍스트
        # ------------------------------------------------------------

        status_marker = Marker()

        status_marker.header.frame_id = self.map_frame
        status_marker.header.stamp = stamp

        status_marker.ns = 'planner_status'
        status_marker.id = marker_id
        status_marker.type = Marker.TEXT_VIEW_FACING
        status_marker.action = Marker.ADD

        status_marker.pose.position.x = boat_x
        status_marker.pose.position.y = boat_y
        status_marker.pose.position.z = 0.85

        status_marker.pose.orientation.w = 1.0

        status_marker.scale.z = 0.24

        status_marker.color = self.make_color(
            1.0,
            1.0,
            1.0,
            1.0,
        )

        status_marker.text = result_text

        marker_array.markers.append(
            status_marker
        )

        self.marker_pub.publish(
            marker_array
        )

        self.last_result_text = result_text

    def publish_status_only(
        self,
        text: str,
    ):
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        self.marker_pub.publish(marker_array)

        if text != self.last_result_text:
            self.get_logger().warn(text)
            self.last_result_text = text

    def make_sphere_marker(
        self,
        marker_id: int,
        namespace: str,
        x: float,
        y: float,
        z: float,
        scale: float,
        color: ColorRGBA,
        stamp,
    ) -> Marker:

        marker = Marker()

        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp

        marker.ns = namespace
        marker.id = marker_id

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        marker.pose.orientation.w = 1.0

        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color = color

        return marker

    def make_line_marker(
        self,
        marker_id: int,
        namespace: str,
        trajectory: List[Tuple[float, float, float]],
        width: float,
        color: ColorRGBA,
        stamp,
    ) -> Marker:

        marker = Marker()

        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp

        marker.ns = namespace
        marker.id = marker_id

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = width
        marker.color = color

        for x, y, _ in trajectory:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.08

            marker.points.append(point)

        return marker

    # ================================================================
    # 보조 함수
    # ================================================================

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(
            math.sin(angle),
            math.cos(angle),
        )

    @staticmethod
    def clamp(
        value: float,
        minimum: float,
        maximum: float,
    ) -> float:
        return max(
            minimum,
            min(maximum, value),
        )

    @staticmethod
    def make_color(
        red: float,
        green: float,
        blue: float,
        alpha: float,
    ) -> ColorRGBA:

        color = ColorRGBA()
        color.r = red
        color.g = green
        color.b = blue
        color.a = alpha

        return color

    @staticmethod
    def yaw_to_quaternion(
        yaw: float,
    ) -> Tuple[float, float, float, float]:

        half_yaw = yaw * 0.5

        return (
            0.0,
            0.0,
            math.sin(half_yaw),
            math.cos(half_yaw),
        )


def main(args=None):
    rclpy.init(args=args)

    node = GapPathVisualizer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
