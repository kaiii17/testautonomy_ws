from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    # ===== 1. 센서 드라이버 계층 =====

    # 라이다 (RPLIDAR S3) - GPS가 ttyUSB0을 쓰므로 라이다는 ttyUSB1 지정
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch', 'sllidar_s3_launch.py'
            )
        ),
        launch_arguments={'serial_port': '/dev/ttyUSB1'}.items()
    )
    ld.add_action(rplidar_launch)

    # 카메라 (ZED2i, zed_wrapper 자체 launch 포함) - usb_cam 2대 구조 폐기,
    # camera_node가 화면 전체에서 색+모양+거리+각도를 한 번에 인식하는 방식으로 변경
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch', 'zed_camera.launch.py'
            )
        ),
        launch_arguments={'camera_model': 'zed2i'}.items()
    )
    ld.add_action(zed_launch)

    # GPS 드라이버 (씨너렉스 SMC-3000) - baudrate 921600 확정 (SMC-2000 아님, 실측 확인됨)
    smc_3000 = Node(
        package='smc_3000',
        executable='smc_3000_node',
        name='smc_3000',
        output='screen',
        parameters=[
            {'device': '/dev/ttyUSB0'},
            {'baudrate': 921600},
        ],
    )
    ld.add_action(smc_3000)

    # TODO: 미션3(도킹)에서 SLAM(Cartographer) 쓰기로 함 - boat_slam_bringup의
    # localization.launch.py(base_link/laser TF, cartographer pure localization,
    # map_server, lifecycle_manager)를 상시 포함할지, 미션3 시작 시점에만 별도
    # 실행할지 팀원과 논의 필요. 포함하기로 하면 여기에 IncludeLaunchDescription
    # 추가.
    #
    # TODO: SLAM에 IMU 데이터를 쓰기로 하면(현재 lua 설정은 use_imu_data=false),
    # smc_imu_converter(kaboat_gps) 노드를 먼저 켜서 /smc3000/imuatta ->
    # sensor_msgs/Imu(/imu)로 변환해줘야 함. 인식 계층에 추가 필요.

    # ===== 2. 인식 계층 (드라이버 켜지고 나서 2초 뒤 시작) =====

    perception_nodes = TimerAction(
        period=2.0,
        actions=[
            Node(package='kaboat_gps', executable='gps_node1',
                 name='gps_node1', output='screen'),
            # color_node -> camera_node로 교체 (ZED2i 통합 인식 노드)
            Node(package='kaboat_camera', executable='camera_node',
                 name='camera_node', output='screen'),
        ]
    )
    ld.add_action(perception_nodes)

    # ===== 3. 판단/제어 계층 (인식 켜지고 나서 3초 뒤 시작) =====

    navigation_nodes = TimerAction(
        period=3.0,
        actions=[
            Node(package='kaboat_navigation', executable='mission_manager',
                 name='mission_manager', output='screen'),
            Node(package='kaboat_navigation', executable='arbiter',
                 name='arbiter', output='screen'),
            Node(package='kaboat_navigation', executable='avoidance',
                 name='avoidance', output='screen'),
            # goal_heading 삭제됨 - 각 미션 노드가 직접 GPS 방위각/거리를
            # 계산하는 구조로 변경 (start/end 템플릿, field_config.py 참고)

            # mission_0(장소이동) 추가 - 출발지에서 미션1 시작 전 준비구역까지
            # 순수 GPS 이동만 담당
            Node(package='kaboat_navigation', executable='mission_0',
                 name='mission_0', output='screen'),
            Node(package='kaboat_navigation', executable='mission_1',
                 name='mission_1', output='screen'),
            Node(package='kaboat_navigation', executable='mission_2',
                 name='mission_2', output='screen'),
            Node(package='kaboat_navigation', executable='mission_3',
                 name='mission_3', output='screen'),
            Node(package='kaboat_navigation', executable='mission_4',
                 name='mission_4', output='screen'),
            Node(package='kaboat_navigation', executable='mission_5',
                 name='mission_5', output='screen'),
            Node(package='kaboat_navigation', executable='thruster_output',
                 name='thruster_output', output='screen'),
        ]
    )
    ld.add_action(navigation_nodes)

    return ld
