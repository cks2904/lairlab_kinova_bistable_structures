import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32, String
import threading

SERIAL_PORT = '/dev/ttyACM0'   # 아두이노 Serial 포트 (환경에 맞게 수정)
BAUD_RATE = 57600

class ServoForcePublisher(Node):
    def __init__(self):
        super().__init__('servo_force_publisher')
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        self.force_pub = self.create_publisher(Float32, 'force_sensor', 10)
        self.state_pub = self.create_publisher(String, 'servo_states', 10)
        
        # 타이머로 센서 데이터 읽기
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        line = self.ser.readline().decode().strip()
        if line.startswith('force'):
            parts = line.split(',')
            if len(parts) == 6:
                try:
                    force_val = float(parts[1])
                    # 소수점 둘째 자리까지 반올림
                    trimmed_force_val = round(force_val, 2)
                    force_msg = Float32()
                    force_msg.data = trimmed_force_val
                    self.force_pub.publish(force_msg)

                    states_msg = String()
                    states_msg.data = ','.join(parts[2:])
                    self.state_pub.publish(states_msg)
                except ValueError:
                    pass

    def user_input_loop(self):
        self.get_logger().info("Servo command input started. Example commands: '1p', '2n', '3u', '4m'")
        while rclpy.ok():
            cmd = input("Enter servo command (e.g. 1p): ").strip()
            if len(cmd) >= 2:
                try:
                    self.ser.write((cmd + '\n').encode())
                    self.get_logger().info(f"Sent command to Arduino: {cmd}")
                except Exception as e:
                    self.get_logger().error(f"Failed to send command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ServoForcePublisher()

    # 사용자 입력을 별도의 스레드에서 비동기로 실행
    input_thread = threading.Thread(target=node.user_input_loop, daemon=True)
    input_thread.start()

    rclpy.spin(node)

    # 종료 전 정리
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

