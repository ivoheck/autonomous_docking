#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#subscriber KLasse https://docs.ros.org/en/kinetic/api/rospy/html/rospy.topics.Subscriber-class.html


from robot_controll_pkg import motor_controller
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from custom_interfaces.msg import DockTrigger
from custom_interfaces.msg import DockFeedback
import math
import time


class FaceWall(Node):

    def __init__(self):
        super().__init__('face_wall')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription 

        self.subscription_node_state = self.create_subscription(
            DockTrigger,
            'dock_trigger/face_wall_trigger',
            self.listener_callback_manage_node_state,
            1)
        self.subscription_node_state

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_node_state = self.create_publisher(DockFeedback, '/dock_feedback', 1)
        self.tolerance = 0.001
        #self.front_limit = 0.1
        self.state = True
        #Diese Variable wird zum Kalibrieren des Lidar-Sensors verwendet
        self.lidar_offset = 0.0008181821216236461
        self.check = 0
        self.checks = 0
        self.controller = motor_controller.MotorControllerHelper()

        self.dock_time = None
        self.node_state = False

        if __name__ == '__main__':
            self.start_node()
            self.dock_time = time.time()


    def listener_callback_manage_node_state(self, msg):
        if msg.trigger:
            self.start_node()
            self.dock_time = msg.time
        else:
            self.stop_node()

    def start_node(self):
        self.node_state = True
        self.time_start = time.time()

    def stop_node(self):
        self.node_state = False
        
    def listener_callback(self, msg):
        if self.node_state == False:
            return
        
        if time.time() - self.time_start > 60.0:
            self.controller.stop()

            msg_dock_feedback = DockFeedback()
            msg_dock_feedback.time = self.dock_time
            msg_dock_feedback.process = 'face_wall'
            msg_dock_feedback.success = False
            msg_dock_feedback.reason = 'time_out'
            
            self.publisher_node_state.publish(msg=msg_dock_feedback)
            self.stop_node()
            return
    
        front = msg.ranges[0]
        l_1, l_2, l_6, l_7, l_11 = msg.ranges[1], msg.ranges[2], msg.ranges[6], msg.ranges[7], msg.ranges[11]
        r_1, r_2, r_6, r_7, r_11 = msg.ranges[719], msg.ranges[718], msg.ranges[714], msg.ranges[715], msg.ranges[709]

        l_value = l_1 + l_2 + l_6 + l_7 + l_11
        r_value = r_1 + r_2 + r_6 + r_7 + r_11

        if math.isinf(abs(l_value)) or math.isinf(abs(r_value)):
            return

        diff = l_value -r_value
        print(l_value,r_value, diff)
        
        diff -= self.lidar_offset

        if abs(diff) < self.tolerance:
            self.controller.stop()
            
            msg_dock_feedback = DockFeedback()
            msg_dock_feedback.time = self.dock_time
            msg_dock_feedback.process = 'face_wall'
            msg_dock_feedback.success = True
            
            self.publisher_node_state.publish(msg=msg_dock_feedback)
            self.stop_node()

        else:
            if diff > 0:
                self.controller.turn_right(percent=3.0)
            else:
                self.controller.turn_left(percent=3.0)
                

def main(args=None):
    rclpy.init(args=args)

    face_wall = FaceWall()

    rclpy.spin(face_wall)
    
    face_wall.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()