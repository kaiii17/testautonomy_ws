from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    bringup_share = FindPackageShare('boat_slam_bringup')

    config_dir = PathJoinSubstitution([
        bringup_share,
        'config'
    ])

    # ==========================================================
    # 1. RPLIDAR S3
    #
    # 실제 라이다를 실행하고 /scan을 발행
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
    # 2. base_footprint -> base_link
    #
    # 배의 2D 중심과 실제 본체 중심 사이의 고정 관계
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
    # 3. base_link -> laser
    #
    # 라이다 설치 위치
    # x=0.30m 앞쪽, z=0.25m 위쪽으로 가정
    # 실제 설치 후 수정
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
    # 4. Cartographer Mapping
    #
    # /scan으로 위치 추정 + 새 지도 작성
    # ==========================================================
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory',
            config_dir,
            '-configuration_basename',
            'boat_cartographer.lua'
        ],
        remappings=[
            ('scan', '/scan')
        ]
    )

    # ==========================================================
    # 5. Occupancy Grid
    #
    # Cartographer 내부 Submap
    #           ↓
    # nav_msgs/OccupancyGrid 형식의 /map
    #
    # RViz Map, Nav2, map_saver_cli가 /map을 사용
    # ==========================================================
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '0.5'
        ]
    )

    # ==========================================================
    # 6. 보트 마커
    # ==========================================================
    boat_marker_node = Node(
        package='boat_slam_bringup',
        executable='boat_marker',
        name='boat_marker_node',
        output='screen'
    )

    # ==========================================================
    # 7. RViz
    # ==========================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        base_link_tf,
        laser_tf,
        boat_marker_node,
        lidar_launch,

        # /scan이 안정화된 후 Cartographer 실행
        TimerAction(
            period=5.0,
            actions=[cartographer_node]
        ),

        # Submap이 생기기 시작한 후 /map 생성
        TimerAction(
            period=8.0,
            actions=[occupancy_grid_node]
        ),

        # 지도 생성 이후 RViz 실행
        TimerAction(
            period=12.0,
            actions=[rviz_node]
        ),
    ])
