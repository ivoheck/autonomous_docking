#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

from robot_controll_pkg import motor_controller
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from custom_interfaces.msg import QrPos
from rclpy.qos import ReliabilityPolicy, QoSProfile
from custom_interfaces.msg import DockTrigger
from custom_interfaces.msg import DockFeedback

class DriveToQr(Node):
    def __init__(self):
        super().__init__('drive_to_qr')
        self.tollerace = 0.05 # value within which straight driving occurs

        self.controller = motor_controller.MotorControllerHelper()
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_node_state = self.create_publisher(DockFeedback, '/dock_feedback', 1)
        self.foundQr = False
        self.state = True
        self.lidar_msg = None

        self.dock_time = None

        self.node_state = False

        self.subscription_trigger = self.create_subscription(
            DockTrigger,
            'trigger_dock_node/drive_to_qr_node',
            self.listener_callback_manage_node_state,
            1)
        self.subscription_trigger

        self.subscription_qr_pos = self.create_subscription(
            QrPos,
            '/qr_pos',
            self.listener_callback,
            1)
        self.subscription_qr_pos

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.lidar_subscription

    def listener_callback_manage_node_state(self,msg):
        if msg.trigger:
            self.start_node()
            self.dock_time = msg.time
        else:
            self.stop_node()


    def start_node(self): 
        self.foundQr = False
        self.state = True
        self.lidar_msg = None

        self.node_state = True
        self.time_start = time.time()


    def stop_node(self):
        self.subscription = None
        self.lidar_subscription = None

        self.foundQr = False
        self.state = True
        self.lidar_msg = None

        self.node_state = False

    def lidar_callback(self,msg):
        self.lidar_msg = msg
        
    def listener_callback(self, msg):
        if self.node_state == False:
            return
        
        if time.time() - self.time_start > 60.0:
            self.controller.stop()

            msg_dock_feedback = DockFeedback()
            msg_dock_feedback.time = self.dock_time
            msg_dock_feedback.process = 'drive_to_qr'
            msg_dock_feedback.success = False
            msg_dock_feedback.reason = 'time_out'
            
            self.publisher_node_state.publish(msg=msg_dock_feedback)
            self.stop_node()
            return
        
        if self.state:
            self.state = False
            self.controller.front(5.0)

        #self.get_logger().info(str(msg))
        if self.lidar_msg is not None:
            front_dist = self.lidar_msg.ranges[0]
            if front_dist <= 0.3:
                self.controller.stop()
                
                msg_dock_feedback = DockFeedback()
                msg_dock_feedback.time = self.dock_time
                msg_dock_feedback.process = 'drive_to_qr'
                msg_dock_feedback.success = True
            
                self.publisher_node_state.publish(msg=msg_dock_feedback)
                self.stop_node()
            
            #self.get_logger().info(str(front_dist))

        direction,offset = msg.direction,msg.offset
        if direction != 0.0:
            self.foundQr = True
            if offset <= self.tollerace:
                self.controller.front(20.0)

            elif offset > self.tollerace:
                self.controller.drive_curve(direction=direction,percent_x=20.0,percent_z=6.0)

        else:
            if self.foundQr and self.lidar_msg is not None:

                #The robot also drives without a QR code until the distance in front becomes too small.
                if front_dist <= 0.3:
                    self.controller.stop()
                    
                    msg_dock_feedback = DockFeedback()
                    msg_dock_feedback.time = self.dock_time
                    msg_dock_feedback.process = 'drive_to_qr'
                    msg_dock_feedback.success = True
            
                    self.publisher_node_state.publish(msg=msg_dock_feedback)
                    self.stop_node()

            #else:
            #    #Wenn nie ein QR code gefunden wurde
            #    self.controller.stop()


def main(args=None):
    rclpy.init(args=args)

    drive_to_qr = DriveToQr()

    rclpy.spin(drive_to_qr)
    
    drive_to_qr.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()