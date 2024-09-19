#https://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
import serial

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64
from builtin_interfaces.msg import Time

from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Twist

import time

class Probe(Node):

    def __init__(self):
        super().__init__('probe')
        self.publisher_ = self.create_publisher(Odometry, '/odometry', 10)

        self.timer = self.create_timer(1, self.pub_odem)

    def pub_odem(self):
        odometry_msg = Odometry()
        header_msg = Header()
        pose_msg = Pose()
        point_msg = Point()
        TwistWithCovariance_msg = TwistWithCovariance()
        twist_msg = Twist()
        time_msg = Time
        

        header_msg.frame_id = 'test'

        current_time_ns = time.time_ns()
        time_msg.sec = current_time_ns // 1_000_000_000  # Ganze Sekunden
        time_msg.nanosec = current_time_ns % 1_000_000_000



        odometry_msg.header = header_msg
        self.publisher_.publish(odometry_msg)

        
def main(args=None):
    rclpy.init(args=args)
    probe = Probe()
    rclpy.spin(probe)
    probe.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()