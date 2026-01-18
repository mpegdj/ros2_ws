import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

# def generate_launch_description():
#     # 패키지 디렉토리 가져오기
#     pkg_my_map = get_package_share_directory('my_map')
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

#     # 월드 파일 절대 경로
#     world_path = os.path.join(pkg_my_map, 'worlds', 'my.world')

#     # 로봇 모델 SDF 절대 경로
#     robot_sdf_path = os.path.join(pkg_my_map, 'models', 'diffbot.sdf')

#     return LaunchDescription([
#         # 1. Gazebo 실행 (내 월드 로드)
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
#             ),
#             launch_arguments={'world': world_path}.items()
#         ),

#         # 2. 로봇 스폰
#         Node(
#             package='gazebo_ros',
#             executable='spawn_entity.py',
#             arguments=['-entity', 'diffbot', '-file', robot_sdf_path],
#             output='screen'
#         )
#     ])


def generate_launch_description():
    # 패키지 디렉토리 가져오기
    pkg_my_map = get_package_share_directory('my_map')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    # 1. URDF 파일 경로 설정
    urdf_file_path = os.path.join(pkg_my_map, 'models', 'diffbot.urdf')
    # 2. 파일 내용을 직접 읽기
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()
    # 3. 노드 실행 시 'parameters'로 전달
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,  # 파일 경로가 아닌 '내용'을 전달
            'use_sim_time': True
        }]
    )
    # 파일 경로 설정
    world_path = os.path.join(pkg_my_map, 'worlds', 'my.world')
    robot_sdf_path = os.path.join(pkg_my_map, 'models', 'diffbot.sdf')
    # URDF 파일 경로 (사용자님 경로 반영)
    urdf_file_path = os.path.join(pkg_my_map, 'models', 'diffbot.urdf')

    # 파라미터 설정
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # 1. Gazebo 실행
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'use_sim_time': use_sim_time}.items()
    )

    # 2. 로봇 스폰 (SDF 기반)
    spawn_diffbot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'diffbot', '-file', robot_sdf_path],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] 
    )

    # 3. Robot State Publisher 추가 (URDF 기반으로 TF 발행)
    # 직접 URDF 파일을 읽어서 robot_description 파라미터로 전달합니다.
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    ld = LaunchDescription()

    # 액션 추가
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(gazebo_cmd)
    ld.add_action(spawn_diffbot_cmd)
    ld.add_action(robot_state_publisher_cmd)

    return ld
