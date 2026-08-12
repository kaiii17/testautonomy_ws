from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ==========================================================
    # 패키지 경로
    # ==========================================================
    bringup_share = FindPackageShare('boat_slam_bringup')

    config_dir = PathJoinSubstitution([
        bringup_share,
        'config'
    ])

    # ==========================================================
    # 실행 인자 1: Linux 사용자 이름
    #
    # 기본값:
    #   kbm11
    # ==========================================================
    user_name_arg = DeclareLaunchArgument(
        'user_name',
        default_value='kbm11',
        description='Linux user name'
    )

    # ==========================================================
    # 실행 인자 2: Cartographer pbstream 파일
    #
    # 역할:
    #   Cartographer가 기존 지도에서 현재 위치를 찾을 때 사용
    #
    # 실행 시:
    #   map_file:=/경로/지도.pbstream
    # ==========================================================
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([
            '/home',
            LaunchConfiguration('user_name'),
            'ros2_ws',
            'maps',
            'lab_map.pbstream'
        ]),
        description='Cartographer pbstream map file'
    )

    # ==========================================================
    # 실행 인자 3: 고정 지도 YAML 파일
    #
    # 역할:
    #   Nav2 Map Server가 PGM 지도를 읽어서 /map으로 발행
    #
    # 실행 시:
    #   map_yaml:=/경로/지도.yaml
    # ==========================================================
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=PathJoinSubstitution([
            '/home',
            LaunchConfiguration('user_name'),
            'ros2_ws',
            'maps',
            'lab_map.yaml'
        ]),
        description='Fixed occupancy-grid YAML map file'
    )

    map_file = LaunchConfiguration('map_file')
    map_yaml = LaunchConfiguration('map_yaml')

    # ==========================================================
    # 1. RPLIDAR S3 실행
    #
    # 결과:
    #   /scan 발행
    # ==========================================================
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'launch',
                'sllidar_s3_launch.py'
            ])
        )
    )

    # ==========================================================
    # 2. base_footprint -> base_link 정적 TF
    #
    # base_footprint:
    #   배의 2D 바닥 중심 좌표
    #
    # base_link:
    #   배 본체 중심 좌표
    # ==========================================================
    base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_static_tf',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.10',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link'
        ]
    )

    # ==========================================================
    # 3. base_link -> laser 정적 TF
    #
    # 아래 값은 라이다가:
    #   배 중심보다 0.30m 앞
    #   배 중심보다 0.25m 위
    #
    # 라는 가정이다.
    #
    # 실제 설치 위치와 다르면 반드시 수정한다.
    # ==========================================================
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_static_tf',
        output='screen',
        arguments=[
            '--x', '0.30',
            '--y', '0.0',
            '--z', '0.25',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser'
        ]
    )

    # ==========================================================
    # 4. Cartographer Pure Localization
    #
    # boat_localization.lua:
    #   pure localization 설정
    #
    # -load_state_filename:
    #   저장된 pbstream 불러오기
    #
    # -load_frozen_state true:
    #   기존 pbstream trajectory를 고정 지도처럼 사용
    #
    # 결과:
    #   map -> odom -> base_footprint TF 계산
    # ==========================================================
    cartographer_localization_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory',
            config_dir,

            '-configuration_basename',
            'boat_localization.lua',

            '-load_state_filename',
            map_file,

            '-load_frozen_state',
            'true'
        ],
        remappings=[
            ('scan', '/scan')
        ]
    )

    # ==========================================================
    # 5. 고정 지도 Map Server
    #
    # YAML 파일을 읽으면 YAML 안의 image 항목을 통해
    # 같은 폴더의 PGM 파일을 자동으로 읽는다.
    #
    # 결과:
    #   /map
    #   /map_metadata
    #
    # 이 /map은 저장된 지도이므로 위치추정 중 바뀌지 않는다.
    # ==========================================================
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': map_yaml,
                'topic_name': 'map',
                'frame_id': 'map',
            }
        ]
    )

    # ==========================================================
    # 6. Map Server Lifecycle Manager
    #
    # map_server는 ROS2 Lifecycle Node라서
    # 실행만 해서는 바로 /map을 발행하지 않는다.
    #
    # lifecycle_manager가 자동으로:
    #   unconfigured
    #       ↓
    #   inactive
    #       ↓
    #   active
    #
    # 상태로 바꿔준다.
    # ==========================================================
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {
                'autostart': True,
                'node_names': ['map_server'],
                'bond_timeout': 4.0,
            }
        ]
    )

    # ==========================================================
    # 7. 보트 마커
    #
    # base_footprint에 붙어 있는 RViz 마커
    #
    # Cartographer가 위치를 계산하면
    # 지도 위에서 보트 마커가 이동한다.
    # ==========================================================
    boat_marker_node = Node(
        package='boat_slam_bringup',
        executable='boat_marker',
        name='boat_marker_node',
        output='screen'
    )

    # ==========================================================
    # 8. RViz
    # ==========================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # ==========================================================
    # 실행 순서
    #
    # 즉시:
    #   정적 TF
    #   라이다
    #   보트 마커
    #
    # 3초:
    #   Map Server
    #
    # 5초:
    #   Lifecycle Manager
    #
    # 7초:
    #   Cartographer Localization
    #
    # 12초:
    #   RViz
    # ==========================================================
    return LaunchDescription([

        user_name_arg,
        map_file_arg,
        map_yaml_arg,

        base_link_tf,
        laser_tf,
        boat_marker_node,
        lidar_launch,

        TimerAction(
            period=3.0,
            actions=[
                map_server_node
            ]
        ),

        TimerAction(
            period=5.0,
            actions=[
                lifecycle_manager_node
            ]
        ),

        TimerAction(
            period=7.0,
            actions=[
                cartographer_localization_node
            ]
        ),

        TimerAction(
            period=12.0,
            actions=[
                rviz_node
            ]
        ),
    ])
