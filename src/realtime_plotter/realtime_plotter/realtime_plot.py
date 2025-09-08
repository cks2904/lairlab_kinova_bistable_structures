import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class RealtimePlotNode(Node):
    def __init__(self):
        super().__init__('realtime_plot_node')

        # 데이터 저장 리스트
        self.distance_data = []
        self.force_data = []

        # Subscriber 설정
        self.sub_distance = self.create_subscription(
            Float32,
            '/gripper_distance',
            self.distance_callback,
            10
        )
        self.sub_force = self.create_subscription(
            Float32,
            '/force_sensor',
            self.force_callback,
            10
        )

        # 최신 값 저장용 변수
        self.current_distance = None
        self.current_force = None

        # Matplotlib Figure 설정
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'bo-')
        self.ax.set_xlabel('Gripper Distance')
        self.ax.set_ylabel('Force')
        self.ax.set_title('Realtime Distance vs Force')

        # 애니메이션 실행
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=200)

    def distance_callback(self, msg):
        self.current_distance = msg.data

    def force_callback(self, msg):
        self.current_force = msg.data

    def update_plot(self, frame):
        if self.current_distance is not None and self.current_force is not None:
            self.distance_data.append(self.current_distance)
            self.force_data.append(self.current_force)

            self.line.set_data(self.distance_data, self.force_data)
            self.ax.relim()
            self.ax.autoscale_view()

        return self.line,

def main(args=None):
    rclpy.init(args=args)
    node = RealtimePlotNode()

    # rclpy는 별도 스레드에서 실행
    from threading import Thread
    def spin():
        rclpy.spin(node)
    t = Thread(target=spin, daemon=True)
    t.start()

    plt.show()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

