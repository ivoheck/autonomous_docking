#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

import rclpy
import time
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class FindQr(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/qr_pos',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.qr_pos = ''
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.callc_movement()
        
    def listener_callback(self, msg):
        self.qr_pos = msg.data
        print(self.qr_pos)

    def callc_movement(self):
        if self.qr_pos == 'left':
            self.turn_left()
            time.sleep(3)

        elif self.qr_pos == 'rigth':
            self.turn_right()
            time.sleep(3)
            
        else:
            self.turn_left()
            time.sleep(3)


    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.2
        self.publisher_.publish(msg)
        time.sleep(1)
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def turn_rigth(self):
        msg = Twist()
        msg.angular.z = -0.2
        self.publisher_.publish(msg)
        time.sleep(1)
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    find_qr = FindQr()

    rclpy.spin(find_qr)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    find_qr.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()