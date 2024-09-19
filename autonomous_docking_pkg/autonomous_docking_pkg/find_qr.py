#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

from autonomous_docking_pkg import motor_controller
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class FindQr(Node):

    def __init__(self):
        super().__init__('find_qr')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/qr_pos',
            self.listener_callback,
            10)
        self.state = True
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.controller = motor_controller.MotorControllerHelper()
        
    def listener_callback(self, msg):
        print(msg.data)
        if self.state:
            if msg.data[0] == 1.0:
                if msg.data[1] < 0.2:
                    self.controller.stop()
                    self.state = False
                elif msg.data[1] < 0.6:
                    self.controller.turn_left(10.0)
                else:
                    self.controller.turn_left(20.0)

            elif msg.data[0] == -1.0:
                if msg.data[1] < 0.2:
                    self.stop()
                elif msg.data[1] < 0.6:
                    self.controller.turn_right(10.0)
                else:
                    self.controller.turn_right(20.0)
            
            else:
                self.controller.turn_left(20.0)
        else:
            print('QR Found')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    find_qr = FindQr()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(find_qr)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            find_qr.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()