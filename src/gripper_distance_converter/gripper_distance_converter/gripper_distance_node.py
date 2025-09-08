import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

class GripperDistanceNode(Node):
    def __init__(self):
        super().__init__('gripper_distance_node')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Float32, '/gripper_moved_distance', 10)
        self.max_position = 0.8
        self.max_distance_mm = 85.2  # 완전 열림에서 정확히 85.2mm

    def listener_callback(self, msg):
        if 'robotiq_85_left_knuckle_joint' in msg.name:
            idx = msg.name.index('robotiq_85_left_knuckle_joint')
            position = msg.position[idx]
            if position != position:  # NaN 체크
                self.get_logger().warn('Position value is NaN')
                return
            current_distance = self.max_distance_mm * (1 - position / self.max_position)
            moved_distance = self.max_distance_mm - current_distance  # 이동한 거리 계산
            dist_msg = Float32()
            dist_msg.data = moved_distance
            self.publisher.publish(dist_msg)
            self.get_logger().info(f"Position: {position:.3f}, Moved Distance: {moved_distance:.2f} mm")
        else:
            self.get_logger().warn('Joint "robotiq_85_left_knuckle_joint" not found in message')

def main(args=None):
    rclpy.init(args=args)
    node = GripperDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

