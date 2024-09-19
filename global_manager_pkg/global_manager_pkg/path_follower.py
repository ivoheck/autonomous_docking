import os
import rclpy
import math
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as R
from autonomous_docking_pkg import motor_controller

class PathFollower(Node):

    def __init__(self):
        super().__init__('start_navigation')
        self.subscription_path = self.create_subscription(
            Path,
            '/path',
            self.listener_callback,
            1)
        self.subscription

        self.subscription_pose = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            1)
        self.subscription

        self.controller = motor_controller.MotorControllerHelper()

        self.pose = None
        self.last_path_pose = None

        self.refresh_rate = 5
        self.timer = self.create_timer(1/self.refresh_rate, self.path_follower_callback)
        
    def listener_callback(self,msg):
        self.last_path_pose = msg.poses[0]

    def listener_callback_odom(self,msg):
        self.pose = msg.pose.pose

    def check_for_orientation_dis(self,pose,last_path_pose):
        current = pose.orientation
        #LLM
        rotation = [current.x,current.y,current.z,current.w]
        rotation = R.from_quat(rotation)
        current = rotation.as_euler('xyz', degrees=False)[2]
        #LLM

        if current < 0:
            current = 2 * math.pi + current

        x1 = pose.position.x
        y1 = pose.position.y

        x2 = last_path_pose.position.x
        y2 = last_path_pose.position.y

        #LLM
        dx = x2 - x1
        dy = y2 - y1

        current = np.arctan2(dx, dy)

        point1 = np.array([x1, y1])
        point2 = np.array([x2, y2])

        distance = np.linalg.norm(point1 - point2)
        #LLM
        return next - current,distance


    def path_folloer_callback(self):
        dif,dis = self.check_for_orientation_dis(self.pose,self.last_path_pose)

        if abs(dif) >= 5.0:
            if dif > 0:
                self.controller.turn_left(50)
            else:
                self.controller.turn_right(50)

        
        if dis > 0.1:
            self.controller.front()
        else:
            self.controller.stop()

def main(args=None):
    rclpy.init(args=args)

    path_follower = PathFollower()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(path_follower)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            path_follower.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()