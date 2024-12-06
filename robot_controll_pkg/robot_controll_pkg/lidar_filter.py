#https://docs.hello-robot.com/0.2/stretch-tutorials/ros2/example_2/

from math import inf
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import itertools

class LidarFilter(Node):

    def __init__(self):
        super().__init__('lidar_filter')
        self.publisher_lidar = self.create_publisher(LaserScan, '/filter_scan', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription
        
    def listener_callback(self,msg):
        filterScan = LaserScan()
        ranges = msg.ranges

        if len(ranges) < 647:
            return

        #75 - 105, 255 - 285, 435 - 465, 615 - 645 filltern
        for i in itertools.chain(range(75,106),range(255,286),range(435,466),range(615,646)):
            ranges[i] = inf

        filterScan.header = msg.header
        filterScan.angle_min = msg.angle_min
        filterScan.angle_max = msg.angle_max 
        filterScan.angle_increment = msg.angle_increment
        filterScan.time_increment = msg.time_increment
        filterScan.scan_time = msg.scan_time
        filterScan.range_min = msg.range_min
        filterScan.range_max = msg.range_max
        filterScan.ranges = ranges
        filterScan.intensities = msg.intensities

        self.publisher_lidar.publish(filterScan)


def main(args=None):
    rclpy.init(args=args)
    lidar_filter = LidarFilter()
    rclpy.spin(lidar_filter)
    lidar_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()