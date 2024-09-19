#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

from autonomous_docking_pkg import motor_controller
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import time

class CenterQr(Node):
    def __init__(self):
        super().__init__('center_qr')
        self.last_direction = None
        self.last_diff = None
        self.offset = 0.8
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/qr_pos',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.controller = motor_controller.MotorControllerHelper()
        
    def listener_callback(self, msg):
        print(msg.data)
        direction,diff = msg.data[0],msg.data[1]
        if direction != 0.0:
            self.last_direction = direction
            self.last_diff = diff
            self.drive(direction=direction,diff=diff)
        else:
            if self.last_direction is not None and self.last_diff is not None:
                self.drive(direction=self.last_direction,diff=self.last_diff)

    def drive(self,direction,diff):
        if diff < 0.05:
            print('end')
            self.controller.stop()
            self.cancel_offset()
            rclpy.shutdown()
            return

        if direction == 1.0:
            self.controller.left(percent=5.0)
        elif direction == -1.0:
            self.controller.right(percent=5.0)
        else:
            self.controller.stop()  

    def cancel_offset(self):
        if self.offset < 0:
            self.controller.right(percent=5.0)
        else:
            self.controller.left(percent=5.0)

        self.get_logger().info('Canceling offset')
        time.sleep(abs(self.offset))
        self.controller.stop()

def main(args=None):
    rclpy.init(args=args)

    center_qr = CenterQr()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(center_qr)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            center_qr.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()