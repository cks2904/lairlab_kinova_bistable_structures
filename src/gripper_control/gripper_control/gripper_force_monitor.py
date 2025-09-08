#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class GripperForceMonitor(Node):
    def __init__(self):
        super().__init__('gripper_force_monitor')
        self._action_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')
        self.get_logger().info("Waiting for gripper action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Gripper action server ready.")
        self.get_logger().info("Interactive Gripper Force Monitor (position [0-0.8], max_effort [0-100])")
        self.get_logger().info("Type '-1' to quit")

    def send_goal(self, position, max_effort):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort
        self._action_client.send_goal_async(goal_msg)
        # 추정 힘 계산 (퍼센트 → N)
        estimated_force = max_effort / 100.0 * 100  # 최대 100N 기준 예시
        self.get_logger().info(f"Gripper moved to {position:.2f} with max_effort={max_effort:.1f}%, estimated_force={estimated_force:.1f} N")

def main(args=None):
    rclpy.init(args=args)
    node = GripperForceMonitor()
    try:
        while True:
            try:
                user_input = input("Input (pos max_effort): ")
                if user_input.strip() == "-1":
                    break
                pos_str, effort_str = user_input.split()
                pos = float(pos_str)
                effort = float(effort_str)
                node.send_goal(pos, effort)
            except ValueError:
                node.get_logger().warn("Invalid input. Use: position max_effort")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

