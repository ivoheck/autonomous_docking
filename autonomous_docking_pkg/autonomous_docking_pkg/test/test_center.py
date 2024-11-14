#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

from autonomous_docking_pkg import motor_controller
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from custom_interfaces.msg import QrPos
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

import time

#TODO: noch nach vorne schauen um zu sehen ob er sich wirklich nach links oder rechts bewegt
class CenterQr(Node):
    def __init__(self):
        super().__init__('center_qr')
        self.last_direction = None
        self.last_diff = None

        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback_lidar,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription_lidar  # prevent unused variable warning

        self.lidar_front = None
        self.lidar_left = None
        self.lidar_right = None

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        self.controller = motor_controller.MotorControllerHelper()

        self.goal_offset = -0.058595824241638186

    def listener_callback_lidar(self, msg):
        self.lidar_front = msg.ranges[0]
        self.lidar_left = msg.ranges[11]
        self.lidar_right = msg.ranges[709]

        print(self.lidar_left - self.lidar_front)

        self.controller.left(percent=5.0)
        


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