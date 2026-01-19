#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    my_map_pkg = get_package_share_directory('my_map')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    
    # 파일 경로
    world_launch_file = os.path.join(my_map_pkg, 'launch', 'my_world.launch.py')
    map_yaml_file = '/home/dtv/ros2_ws/src/my_map/maps/my_world_map.yaml'
    nav2_params_file = '/home/dtv/ros2_ws/src/my_map/params/nav2_params.yaml'
    
    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 1. World 실행 (my_world.launch.py)
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch_file),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 2. Nav2 Bringup 실행 (3초 후)
    nav2_bringup_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_yaml_file,
                    'params_file': nav2_params_file,
                }.items()
            )
        ]
    )
    
    # 3. 초기 위치 설정 (6초 후 - AMCL 준비 대기)
    initial_pose_node = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, '
                    'pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, '
                    'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, '
                    'covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, '
                    '0.0, 0.25, 0.0, 0.0, 0.0, 0.0, '
                    '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
                    '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
                    '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
                    '0.0, 0.0, 0.0, 0.0, 0.0, 0.07]}}'
                ],
                shell=False
            )
        ]
    )
    
    # 4. RViz2 실행 (8초 후 - 모든 것이 준비된 후)
    rviz_launch = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(nav2_bringup_pkg, 'rviz', 'nav2_default_view.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # 순서대로 실행
        world_launch,           # 즉시 실행
        nav2_bringup_launch,    # 3초 후
        initial_pose_node,      # 6초 후
        rviz_launch,           # 8초 후
    ])