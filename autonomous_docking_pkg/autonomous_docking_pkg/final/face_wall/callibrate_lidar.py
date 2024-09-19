#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

#subscriber KLasse https://docs.ros.org/en/kinetic/api/rospy/html/rospy.topics.Subscriber-class.html

#Der Roboter wird vor eine gerade wand plaziert so das er möglichst genau in 90 grad zu ihr steht
#Danach kann mit diesem script der offset berechnet werden der nötig ist um den lidar zu kallibrirens
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

import math


class CalibrateLidar(Node):

    def __init__(self):
        super().__init__('calibrate_lidar')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription  # prevent unused variable warning
        self.state = 0
        self.diff_list = []
        
    def listener_callback(self, msg):
        if self.state >= 11:
            res = 0

            for i in self.diff_list:
                res += i
            
            res /= len(self.diff_list)
            print(res)
            rclpy.shutdown()

        #roboter ist rückwerts
        front = msg.ranges[0]
        l_1, l_2, l_6, l_7, l_11 = msg.ranges[1], msg.ranges[2], msg.ranges[6], msg.ranges[7], msg.ranges[11]
        r_1, r_2, r_6, r_7, r_11 = msg.ranges[719], msg.ranges[718], msg.ranges[714], msg.ranges[715], msg.ranges[709]

        l_value = l_1 + l_2 + l_6 + l_7 + l_11
        r_value = r_1 + r_2 + r_6 + r_7 + r_11
        
        if not math.isinf(l_value) and not math.isinf(r_value):

            diff = l_value -r_value
            self.diff_list.append(diff)
            self.state += 1
        

def main(args=None):
    rclpy.init(args=args)

    calibrate_lidar = CalibrateLidar()

    rclpy.spin(calibrate_lidar)

    calibrate_lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()