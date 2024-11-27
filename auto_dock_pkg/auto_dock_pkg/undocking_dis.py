#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

from robot_controll_pkg import motor_controller
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String
from math import inf

#0.14 bis chassi

class Undock(Node):

    def __init__(self):
        super().__init__('undock')
        self.controller = motor_controller.MotorControllerHelper()
        self.publisher_lock = self.create_publisher(Bool, '/lock', 1)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription  
        self.lidar_front = None
        self.status = 0
        self.lidar_offset = 0.14

        #Zum manulellen undocking
        if __name__ == '__main__':
            self.open_lock()
            self.timer = self.create_timer(0.1, self.start_undock)

    def start_undock(self):
        print(self.lidar_front)
        if self.lidar_front is None or self.lidar_front == inf:
            return

        if self.lidar_front > 0.7 + self.lidar_offset:
            print('final: ',self.lidar_front - self.lidar_offset)
            self.controller.stop()
            self.close_lock()
            self.timer.destroy()
            
        else:
            self.controller.back(10.0)


    def listener_callback(self,msg):
        self.lidar_front = msg.ranges[0]

    def open_lock(self):
        msg = Bool()
        msg.data = True
        self.publisher_lock.publish(msg)

    def close_lock(self):
        msg = Bool()
        msg.data = False
        self.publisher_lock.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    undock = Undock()
    rclpy.spin(undock)
    
    undock.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()