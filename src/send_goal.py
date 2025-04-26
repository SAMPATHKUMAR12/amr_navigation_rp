#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_done = False

    def send_goal(self, x, y, w=1.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = w  # face forward (no rotation)

        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected 😞')
            self.goal_done = True
            return

        self.get_logger().info('Goal accepted! 🚀')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation finished with status: {future.result().status}')
        self.goal_done = True

def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    node.send_goal(x=1.0, y=1.0)  # Change coordinates here

    while rclpy.ok() and not node.goal_done:
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
