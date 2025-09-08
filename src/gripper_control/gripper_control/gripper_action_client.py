#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import sys
import time

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control_node')
        self._action_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')
        self.get_logger().info("Waiting for gripper action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Gripper action server ready.")
        self.interactive_control()

    def send_goal_sync(self, position, max_effort):
        """
        position: 0.0(open) ~ 0.8(close)
        max_effort: % 단위
        """
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        # 액션 완료 후 추정 힘 계산
        estimated_force = self.estimate_force(position, max_effort)
        self.get_logger().info(f"Gripper moved to {position:.2f} with max_effort={max_effort:.1f}%, estimated_force={estimated_force:.1f}%")

    def estimate_force(self, position, max_effort):
        """
        센서 없는 경우 상대적 힘 추정
        0% ~ max_effort 비율
        """
        # gripper 0.0~0.8 이동 범위 기준
        gripper_range = 0.8
        # 물체 접촉 후 위치 변화율 감소 => 힘 증가
        relative_gap_ratio = 1.0 - min(position / gripper_range, 1.0)
        estimated_force = max_effort * relative_gap_ratio
        return estimated_force

    def interactive_control(self):
        self.get_logger().info("Interactive Gripper Control")
        self.get_logger().info("Enter: position max_effort(%)  (e.g., 0.5 50)")
        self.get_logger().info("Type '-1' to quit")
        while True:
            try:
                user_input = input("Input (pos max_effort): ")
                if user_input.strip() == '-1':
                    break
                position_str, max_effort_str = user_input.strip().split()
                position = float(position_str)
                max_effort = float(max_effort_str)
                self.send_goal_sync(position, max_effort)
                time.sleep(0.1)  # 액션 후 짧은 딜레이
            except Exception as e:
                self.get_logger().error(f"Invalid input or error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

