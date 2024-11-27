#https://www.theconstruct.ai/ros-qa-196-how-to-output-odometry-data/

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from scipy.spatial.transform import Rotation as R
import time
import math

#Class listens to the cmd_vel topic ang generates and publishes the odometry inormation
class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')
        self.refresh_rate = 10 #30

        self.twist = None
        self.last_z_rad = 0.0
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0

        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/encoder',
            self.listener_callback_encoder,
            1)
        self.subscription_cmd_vel 

        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        #10 herz
        self.timer = self.create_timer(1/self.refresh_rate, self.odom_callback)


    def listener_callback_encoder(self, msg):
        self.twist = msg

    def odom_callback(self):
        odometry_msg = Odometry()

        odometry_msg.header.frame_id = 'odom'
        odometry_msg.child_frame_id = 'base_link'

        if self.twist is not None:
            odometry_msg.twist.twist = self.twist
        
            #Winkel geht von -pi bist +pi
            rad_per_second = self.twist.angular.z
            rad = rad_per_second * (1/self.refresh_rate)
            self.last_z_rad += rad

            if self.last_z_rad > math.pi:
                self.last_z_rad = -2 * math.pi + self.last_z_rad

            elif self.last_z_rad < -math.pi:
                self.last_z_rad = 2 * math.pi + self.last_z_rad
            
            rotation = R.from_euler('xyz', [0,0,self.last_z_rad], degrees=False)
            quaternion = rotation.as_quat()
            odometry_msg.pose.pose.orientation.x = quaternion[0]
            odometry_msg.pose.pose.orientation.y = quaternion[1]
            odometry_msg.pose.pose.orientation.z = quaternion[2]
            odometry_msg.pose.pose.orientation.w = quaternion[3]

            x_m_per_second = self.twist.linear.x
            x_m = x_m_per_second * (1/self.refresh_rate)
            y_from_x = math.sin(self.last_z_rad) * x_m
            x_from_x = math.cos(self.last_z_rad) * x_m

            y_m_per_second = self.twist.linear.y
            y_m = y_m_per_second * (1/self.refresh_rate)
            x_from_y = math.sin(self.last_z_rad) * y_m
            y_from_y = math.cos(self.last_z_rad) * y_m

            self.last_pose_x += x_from_x + x_from_y
            self.last_pose_y += y_from_x + y_from_y

            odometry_msg.pose.pose.position.x = self.last_pose_x
            odometry_msg.pose.pose.position.y = self.last_pose_y

        #LLM
        current_time = self.get_clock().now().to_msg()
        odometry_msg.header.stamp = current_time
        self.publisher_.publish(odometry_msg)


def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()