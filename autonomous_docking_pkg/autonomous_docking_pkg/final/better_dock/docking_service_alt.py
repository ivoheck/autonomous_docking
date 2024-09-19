from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Int32MultiArray
import json


class DockingService(Node):

    def __init__(self):
        super().__init__('docking_service')
        self.srv = self.create_service(Trigger, '/docking_service', self.docking_service)

        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription_lidar  # prevent unused variable warning
        self.lidar_front = None

        self.subscription_probe = self.create_subscription(
            Int32MultiArray,
            '/probe',
            self.probe_callback,
            10)
        self.subscription_probe  # prevent unused variable warning
        self.probe = []
        

    def docking_service(self,request, response):
        self.get_logger().info('Incoming request')
        response.message = json.dumps([self.lidar_front,self.probe])
        return response

    def lidar_callback(self,msg):
        self.lidar_front = msg.ranges[0]

    def probe_callback(self,msg):
        self.probe = [msg.data[0],msg.data[1]]


def main(args=None):
    rclpy.init(args=args)

    docking_service = DockingService()

    rclpy.spin(docking_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()