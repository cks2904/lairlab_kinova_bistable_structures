import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import serial
import threading
import time

class ArduinoServoNode(Node):
    """
    Handles communication with an Arduino for servo control.
    - Subscribes to 'servo_cmd' (std_msgs/String) to receive commands.
    - Publishes feedback from Arduino to 'servo_feedback' (std_msgs/Float64MultiArray).
    - Allows direct command input from the terminal.
    """
    def __init__(self):
        super().__init__('arduino_servo_node')
        
        # --- Use ROS 2 Parameters for flexible configuration ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        # --- End of parameter section ---

        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"Connected to Arduino on {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            rclpy.shutdown()
            exit(1)

        self.sub = self.create_subscription(String, 'servo_cmd', self.command_callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, 'servo_feedback', 10)
        threading.Thread(target=self.read_and_publish_loop, daemon=True).start()
        threading.Thread(target=self.user_input_loop, daemon=True).start()

    def command_callback(self, msg: String):
        cmd = msg.data.strip()
        if cmd:
            self.send_to_arduino(cmd)

    def user_input_loop(self):
        self.get_logger().info("Ready for servo commands (e.g., '1n 2p 3u 4m').")
        while rclpy.ok():
            try:
                cmd = input().strip()
                if cmd:
                    self.send_to_arduino(cmd)
            except EOFError:
                break

    def send_to_arduino(self, cmd: str):
        try:
            self.ser.write((cmd + '\n').encode())
            self.get_logger().info(f"Sent: {cmd}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def read_and_publish_loop(self):
        while rclpy.ok():
            if not self.ser.is_open:
                break
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    try:
                        float_values = [float(val) for val in line.split()]
                        if float_values:
                            msg = Float64MultiArray(data=float_values)
                            self.pub.publish(msg)
                    except ValueError:
                        pass
            except Exception as e:
                self.get_logger().error(f"Error in read/publish loop: {e}")
                break

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
            node.get_logger().info("Serial port closed.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
