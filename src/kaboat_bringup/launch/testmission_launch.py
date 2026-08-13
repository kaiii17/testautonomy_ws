from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    단일 미션 테스트용 launch 파일.
    정식 mission_manager 대신 test_mission_manager를 써서 지정한 미션
    하나만 계속 active 상태로 유지한다. 센서 드라이버 + arbiter/avoidance/
    thruster_output은 실전과 동일하게 다 켜지고, mission_1~5 중 지정한
    미션 노드 하나만 실행된다.

    사용법 (기본값은 mission_3):
      ros2 launch kaboat_bringup kaboat_test_mission.launch.py
      ros2 launch kaboat_bringup kaboat_test_mission.launch.py mission:=mission_4
    """

    mission_arg = DeclareLaunchArgument(
        'mission', default_value='mission_3',
        description='단독 테스트할 미션 이름 (mission_1 ~ mission_5 중 하나)'
    )
    mission = LaunchConfiguration('mission')

    ld = LaunchDescription([mission_arg])

    # ===== 1. 센서 드라이버 계층 (bringup과 동일) =====

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

    # ===== 2. 인식 계층 (드라이버 켜지고 나서 2초 뒤 시작) =====

    perception_nodes = TimerAction(
        period=2.0,
        actions=[
            Node(package='kaboat_gps', executable='gps_node1',
                 name='gps_node1', output='screen'),
            Node(package='kaboat_camera', executable='camera_node',
                 name='camera_node', output='screen'),
        ]
    )
    ld.add_action(perception_nodes)

    # ===== 3. 판단/제어 계층 - 테스트 매니저 + arbiter/avoidance/thruster
    #          + 지정한 미션 노드 하나만 (조건부 실행) =====

    core_nodes = TimerAction(
        period=3.0,
        actions=[
            Node(package='kaboat_navigation', executable='test_mission_manager',
                 name='test_mission_manager', output='screen',
                 parameters=[{'mission': mission}]),
            Node(package='kaboat_navigation', executable='arbiter',
                 name='arbiter', output='screen'),
            Node(package='kaboat_navigation', executable='avoidance',
                 name='avoidance', output='screen'),
            Node(package='kaboat_navigation', executable='thruster_output',
                 name='thruster_output', output='screen'),

            # mission 인자값과 일치하는 것 하나만 실제로 실행됨
            Node(package='kaboat_navigation', executable='mission_1',
                 name='mission_1', output='screen',
                 condition=IfCondition(PythonExpression(["'", mission, "' == 'mission_1'"]))),
            Node(package='kaboat_navigation', executable='mission_2',
                 name='mission_2', output='screen',
                 condition=IfCondition(PythonExpression(["'", mission, "' == 'mission_2'"]))),
            Node(package='kaboat_navigation', executable='mission_3',
                 name='mission_3', output='screen',
                 condition=IfCondition(PythonExpression(["'", mission, "' == 'mission_3'"]))),
            Node(package='kaboat_navigation', executable='mission_4',
                 name='mission_4', output='screen',
                 condition=IfCondition(PythonExpression(["'", mission, "' == 'mission_4'"]))),
            Node(package='kaboat_navigation', executable='mission_5',
                 name='mission_5', output='screen',
                 condition=IfCondition(PythonExpression(["'", mission, "' == 'mission_5'"]))),
        ]
    )
    ld.add_action(core_nodes)

    return ld
