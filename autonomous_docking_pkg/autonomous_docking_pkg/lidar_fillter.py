#https://docs.hello-robot.com/0.2/stretch-tutorials/ros2/example_2/

from math import inf
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import itertools

#ab und zu gibt es einen errror beim starten
class LidarFillter(Node):

    def __init__(self):
        super().__init__('lidar_fillter')
        self.publisher_lidar = self.create_publisher(LaserScan, '/fillter_scan', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription
        
    def listener_callback(self,msg):
        fillterScan = LaserScan()
        ranges = msg.ranges

        #75 - 105, 255 - 285, 435 - 465, 615 - 645 filltern
        for i in itertools.chain(range(75,106),range(255,286),range(435,466),range(615,646)):
            ranges[i] = inf

        fillterScan.header = msg.header
        fillterScan.angle_min = msg.angle_min
        fillterScan.angle_max = msg.angle_max 
        fillterScan.angle_increment = msg.angle_increment
        fillterScan.time_increment = msg.time_increment
        fillterScan.scan_time = msg.scan_time
        fillterScan.range_min = msg.range_min
        fillterScan.range_max = msg.range_max
        fillterScan.ranges = ranges
        fillterScan.intensities = msg.intensities

        self.publisher_lidar.publish(fillterScan)


def main(args=None):
    rclpy.init(args=args)

    lidar_fillter = LidarFillter()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(lidar_fillter)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            lidar_fillter.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()