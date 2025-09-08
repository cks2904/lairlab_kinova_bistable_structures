import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from collections import deque

ACTION_NAME = '/robotiq_gripper_controller/gripper_cmd'
CLOSE_POSITION = 0.8
OPEN_POSITION = 0.0

TIMER_PERIOD = 0.2
MOVING_AVG_WINDOW = 20
POSITION_STEP_MAX = 0.0015
POSITION_STEP_MIN = 0.0003
POSITION_CORRECTION_STEP = 0.003
DEFAULT_FORCE_THRESHOLD = 3.0  # 초기 상한값

class GripperForceControl(Node):
    def __init__(self):
        super().__init__('gripper_force_control')

        # 상한값 파라미터만 선언
        self.declare_parameter('threshold_upper', DEFAULT_FORCE_THRESHOLD)
        self.threshold_upper = float(self.get_parameter('threshold_upper').value)

        self.force_values = deque(maxlen=MOVING_AVG_WINDOW)
        self.filtered_force = 0.0

        self.force_sub = self.create_subscription(Float32, 'force_sensor', self.force_callback, 10)

        self._client = ActionClient(self, GripperCommand, ACTION_NAME)
        self.goal_handle = None
        self.canceling = False

        self.current_position = OPEN_POSITION
        self.force_above_threshold = False

        # 실시간 파라미터 변경 이벤트 등록
        self.add_on_set_parameters_callback(self.parameter_update_callback)

        self.timer = self.create_timer(TIMER_PERIOD, self.control_step)

        self.get_logger().info(f"Control timer period set to {TIMER_PERIOD}s")
        self.get_logger().info(f"Use moving average window size: {MOVING_AVG_WINDOW}")
        self.get_logger().info(f"Position step max: {POSITION_STEP_MAX}, min: {POSITION_STEP_MIN}")
        self.get_logger().info(f"Initial threshold_upper: {self.threshold_upper} N")

    def parameter_update_callback(self, params):
        for param in params:
            if param.name == 'threshold_upper':
                self.threshold_upper = param.value
                self.get_logger().info(f"[PARAM] threshold_upper updated: {self.threshold_upper} N")
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    def force_callback(self, msg: Float32):
        self.force_values.append(msg.data)
        if len(self.force_values) > 0:
            self.filtered_force = sum(self.force_values) / len(self.force_values)
        else:
            self.filtered_force = 0.0
        self.get_logger().debug(f"Filtered force: {self.filtered_force:.3f} N")

    def control_step(self):
        # 실시간 파라미터 읽기 (필요하면 추가)
        # self.threshold_upper = self.get_parameter('threshold_upper').value

        if self.force_above_threshold:
            if self.filtered_force < self.threshold_upper * 0.9:  # 임의 완화 기준
                self.force_above_threshold = False
            else:
                return
        else:
            if self.filtered_force >= self.threshold_upper:
                self.force_above_threshold = True
                if self.goal_handle and not self.canceling:
                    self.canceling = True
                    cancel_future = self.goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(self._on_cancel_done)
                self.current_position = max(OPEN_POSITION, self.current_position - POSITION_CORRECTION_STEP)
                self.send_close_goal_once(self.current_position)
                return

        # 그리퍼 위치 증가
        if self.goal_handle is None and not self.canceling:
            if self.filtered_force > (self.threshold_upper * 0.85):
                position_step = POSITION_STEP_MIN
            else:
                position_step = POSITION_STEP_MAX

            if self.current_position < CLOSE_POSITION:
                self.current_position += position_step
                if self.current_position > CLOSE_POSITION:
                    self.current_position = CLOSE_POSITION
                self.send_close_goal_once(self.current_position)

    def send_close_goal_once(self, position: float):
        if not self._client.server_is_ready():
            self.get_logger().info("Waiting for gripper action server...")
            self._client.wait_for_server(timeout_sec=0.5)
            if not self._client.server_is_ready():
                self.get_logger().warn("Gripper action server not ready.")
                return
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        self.get_logger().info(f"Sending gripper goal: position={position:.4f}")
        send_goal_future = self._client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Gripper close goal rejected by server.")
            self.goal_handle = None
            return
        self.get_logger().info("Gripper close goal accepted.")
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self.get_logger().info("Gripper close goal finished.")
        self.goal_handle = None
        self.canceling = False

    def _on_cancel_done(self, future):
        self.get_logger().info("Gripper close goal canceled.")
        self.goal_handle = None
        self.canceling = False

def main(args=None):
    rclpy.init(args=args)
    node = GripperForceControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down on user interrupt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

