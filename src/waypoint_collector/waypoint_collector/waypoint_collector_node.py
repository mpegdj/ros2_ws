#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor # 멀티스레드 추가
import threading

class WaypointCollector(Node):
    def __init__(self):
        super().__init__('waypoint_collector')
        self.waypoints = []
        
        # 구독 설정
        self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )

        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        self.get_logger().info(
            '\n' + '='*40 +
            '\nWaypoint Collector ready.'
            '\n1) RViz: "Publish Point"로 경유지들을 클릭하세요.'
            '\n2) 클릭이 완료되면 터미널에서 ENTER를 누르세요.'
            '\n' + '='*40
        )

    def clicked_point_callback(self, msg):
        # 이제 실시간으로 추가되는 로그를 볼 수 있습니다.
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.orientation.w = 1.0
        self.waypoints.append(pose)
        self.get_logger().info(f'📍 Point 추가됨: ({msg.point.x:.2f}, {msg.point.y:.2f}) [총 {len(self.waypoints)}개]')

    def send_waypoints(self):
        if not self.waypoints:
            self.get_logger().warn('⚠️ 전송할 Waypoint가 없습니다! 먼저 클릭해주세요.')
            return

        self.get_logger().info(f'🚀 서버 연결 대기 중...')
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Nav2 Action Server가 활성화되지 않았습니다!')
            return

        goal = FollowWaypoints.Goal()
        goal.poses = self.waypoints
        self.get_logger().info(f'✅ {len(self.waypoints)}개 포인트를 Nav2로 전송합니다!')
        self.action_client.send_goal_async(goal)

def input_thread(node):
    """엔터 입력을 감시하는 별도 스레드"""
    while rclpy.ok():
        input() # 여기서 대기해도 메인 스핀은 멈추지 않음
        node.send_waypoints()

def main():
    rclpy.init()
    node = WaypointCollector()

    # 멀티스레드 실행기 설정
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # 입력을 기다리는 스레드 시작
    thread = threading.Thread(target=input_thread, args=(node,), daemon=True)
    thread.start()

    try:
        executor.spin() # 메시지 수신(콜백)은 여기서 처리됨
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()