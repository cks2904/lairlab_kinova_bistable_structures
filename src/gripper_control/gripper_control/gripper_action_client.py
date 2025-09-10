#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import threading
import time
 
class GripperController(Node):
    """
    Controls a gripper and provides real-time feedback on its absolute position.
    - Calibrates its zero position by moving to the fully open state on startup.
    - Publishes the travel distance from this calibrated zero position at 10Hz.
    """
 
    def __init__(self):
        super().__init__('gripper_controller_node')
 
        # --- Constants and State Variables ---
        self.MM_PER_POSITION_UNIT = 85.2 / 0.8
        self.GRIPPER_JOINT_NAME = 'robotiq_85_left_knuckle_joint'
        self.calibrated_zero_position = None # The "true zero" learned on startup
        self.current_position = 0.0
        self.data_lock = threading.Lock()
        self.node_ready = threading.Event()
 
        # --- ROS 2 Communications ---
        self._action_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')
        self.distance_publisher = self.create_publisher(Float64, '/gripper_moved_distance', 10)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
 
        # --- Start background threads ---
        threading.Thread(target=self.main_loop, daemon=True).start()
        threading.Thread(target=self.publish_loop, daemon=True).start()
 
    def main_loop(self):
        """Handles the node's lifecycle: calibration and user input."""
        self.get_logger().info("Waiting for gripper action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server available.")
 
        # Step 1: Calibrate the gripper's zero position
        self.calibrate_zero_position()
        self.get_logger().info("Enter position (0.0: open, 0.8: close). Type 'quit' to exit.")
        while rclpy.ok():
            try:
                user_input = input("Enter position: ").strip()
                if user_input.lower() == 'quit':
                    rclpy.shutdown()
                    break
                position = float(user_input)
                if not (0.0 <= position <= 0.8):
                    self.get_logger().warn("Position must be between 0.0 and 0.8.")
                    continue
                self.send_goal(position)
            except Exception:
                break
 
    def calibrate_zero_position(self):
        """Moves gripper to 0.0 and records the actual reported position as the 'true zero'."""
        self.get_logger().info("Calibrating zero position... Moving to state 0.0.")
        self.send_goal(0.0)
        time.sleep(2) # Allow time for the gripper to physically move and settle
 
        with self.data_lock:
            # The current position at this time is our true zero offset
            self.calibrated_zero_position = self.current_position
        self.node_ready.set() # Signal that calibration is done
        self.get_logger().info(f"Calibration complete. Zero position offset is {self.calibrated_zero_position:.4f}")
 
    def joint_state_callback(self, msg: JointState):
        """Updates the current gripper position from /joint_states."""
        try:
            idx = msg.name.index(self.GRIPPER_JOINT_NAME)
            with self.data_lock:
                self.current_position = msg.position[idx]
        except (ValueError, TypeError):
            pass
 
    def publish_loop(self):
        """Continuously publishes the distance from the calibrated zero position."""
        rate = self.create_rate(10) # 10Hz
        self.node_ready.wait() # Wait for calibration to finish
        while rclpy.ok():
            moved_distance_mm = 0.0
            with self.data_lock:
                # Calculate distance relative to the learned zero position
                # This correctly handles any built-in offset in the URDF or driver
                relative_position = self.current_position - self.calibrated_zero_position
                moved_distance_mm = relative_position * self.MM_PER_POSITION_UNIT
            # Ensure distance is not negative due to small float inaccuracies
            msg = Float64(data=max(0.0, moved_distance_mm))
            self.distance_publisher.publish(msg)
            rate.sleep()
 
    def send_goal(self, target_position: float):
        """Sends a position goal to the action server asynchronously."""
        self.get_logger().info(f"Sending goal: position {target_position:.3f}")
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = target_position
        self._action_client.send_goal_async(goal_msg)
 
def main(args=None):
    rclpy.init(args=args)
    controller_node = GripperController()
    try:
        rclpy.spin(controller_node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        controller_node.get_logger().info("Shutting down gripper controller node.")
        controller_node.destroy_node()
 
if __name__ == '__main__':
    main()