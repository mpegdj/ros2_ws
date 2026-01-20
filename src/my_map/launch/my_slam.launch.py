import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_my_map = get_package_share_directory('my_map')

    # 1. 이미 만든 my_world.launch.py를 포함 (Include)
    my_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_my_map, 'launch', 'my_world.launch.py')
        )
    )

    # 2. 여기에 SLAM 노드만 정의하여 추가
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'scan_topic': '/scan',
            'mode': 'mapping',

            # --- 해상도 및 맵 크기 관련 핵심 파라미터 ---
            'resolution': 0.01,          # 0.05에서 0.01로 변경 (더 정밀하고 큰 지도)
            'max_laser_range': 10.0,     # 레이저가 인식할 최대 거리
            'minimum_time_interval': 0.5,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000 # 메모리 에러 방지용
        }]
    )

    ld = LaunchDescription()

    # 세계를 먼저 띄우고, 그 위에 SLAM을 올립니다.
    ld.add_action(my_world_cmd)
    ld.add_action(slam_toolbox_node)

    return ld