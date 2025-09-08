import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

SERIAL_PORT = '/dev/ttyACM0'   # 아두이노 Serial 포트 (환경에 맞게 수정)
BAUD_RATE = 115200              # 아두이노 코드와 맞춰야 함

class ArduinoServoNode(Node):
    def __init__(self):
        super().__init__('arduino_servo_node')

        # 아두이노 시리얼 연결
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {SERIAL_PORT} at {BAUD_RATE}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            exit(1)

        # ROS 2 토픽 구독 (서보 명령 수신)
        self.sub = self.create_subscription(
            String,
            'servo_cmd',
            self.servo_callback,
            10
        )

        # 사용자 입력도 비동기로 가능
        self.input_thread = threading.Thread(target=self.user_input_loop, daemon=True)
        self.input_thread.start()

    def servo_callback(self, msg: String):
        cmd_line = msg.data.strip()
        if cmd_line:
            # 아두이노에 한 줄로 전송
            try:
                self.ser.write((cmd_line + '\n').encode('utf-8'))
                self.get_logger().info(f"Sent to Arduino: {cmd_line}")
            except Exception as e:
                self.get_logger().error(f"Failed to send command: {e}")

    def user_input_loop(self):
        self.get_logger().info("Servo command input started. Example: '1n 2p 3u 4m'")
        while rclpy.ok():
            cmd = input("Enter servo command: ").strip()
            if cmd:
                try:
                    self.ser.write((cmd + '\n').encode('utf-8'))
                    self.get_logger().info(f"Sent command to Arduino: {cmd}")
                except Exception as e:
                    self.get_logger().error(f"Failed to send command: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

