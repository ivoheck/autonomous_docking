#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

from robot_controll_pkg import motor_controller
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from custom_interfaces.msg import QrPos
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from math import inf
from custom_interfaces.msg import DockTrigger
from custom_interfaces.msg import DockFeedback
import time

class CenterQr(Node):
    def __init__(self):
        super().__init__('center_qr')
        self.last_direction = None
        self.last_speed = None
        self.subscription = self.create_subscription(
            QrPos,
            '/qr_pos',
            self.listener_callback,
            1)
        self.subscription 

        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback_lidar,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription_lidar  

        self.subscription_node_state = self.create_subscription(
            DockTrigger,
            'trigger_dock_node/qr_center_node',
            self.listener_callback_manage_node_state,
            1)
        self.subscription_node_state

        self.lidar_front = None
        self.lidar_left = None
        self.lidar_right = None

        self.publisher_node_state = self.create_publisher(DockFeedback, '/dock_feedback', 1)
        self.controller = motor_controller.MotorControllerHelper()

        self.goal_offset = -0.058595824241638186

        self.dock_time = None
        self.node_state = False

        if __name__ == '__main__':
            print('test')
            self.start_node()

    def listener_callback_manage_node_state(self, msg):
        if msg.trigger:
            self.start_node()
            self.dock_time = msg.time
        else:
            self.stop_node()

    def start_node(self):
        self.last_direction = None
        self.last_speed = None
        self.node_state = True
        self.time_start = time.time()
        

    def stop_node(self):
        self.node_state = False
        self.last_direction = None
        self.last_speed = None

    def listener_callback_lidar(self, msg):
        if self.node_state == False:
            return
        
        if time.time() - self.time_start > 60.0:#60
            self.controller.stop()

            msg_dock_feedback = DockFeedback()
            msg_dock_feedback.time = self.dock_time
            msg_dock_feedback.process = 'qr_center'
            msg_dock_feedback.success = False
            msg_dock_feedback.reason = 'time_out'
            
            self.publisher_node_state.publish(msg=msg_dock_feedback)
            self.stop_node()
            return
        
        if msg.ranges[0] != inf:
            self.lidar_front = msg.ranges[0]

        if msg.ranges[21] != inf:
            self.lidar_left = msg.ranges[21]

        if msg.ranges[699] != inf:
            self.lidar_right = msg.ranges[699]
        
    def listener_callback(self, msg):
        if self.node_state == False:
            return
        

        #Abstand zur station wird gegebenfalls korrigiert
        if self.lidar_front is not None and self.lidar_front < 0.25:
            self.controller.back(percent=5.0)
            return
        
        elif self.lidar_front is not None and self.lidar_front > 0.3:
            self.controller.front(percent=5.0)
            return

        self.get_logger().info(str(msg))

        #Überprüfung ob ein QR code in aktueller iteration gefunden wurde
        if msg.qrcode == 'l':
            self.last_direction = -1.0
            self.last_speed = 5.0
            self.controller.right(percent=5.0)
            self.get_logger().debug('rechts')
            return
        
        if msg.qrcode == 'r':
            self.last_direction = 1.0
            self.last_speed = 5.0
            self.controller.left(percent=5.0)
            self.get_logger().debug('links')
            return
        
        if msg.qrcode == 'c':
            self.last_direction,self.last_speed  = self.final_centering(msg.offset)
            return
        

        #Kein qr code in aktueller iteration
        #Hier wird überprüft ob Roboter rechts oder links an der Stion vorbei fährt
        if self.lidar_right is not None and self.lidar_left is not None:
            #Wenn der Roboter zu weit links ist muss das zentrum rechts sein
            self.get_logger().debug(f'offset: {self.lidar_left - self.lidar_right}')
            if self.lidar_left - self.lidar_right > 0.045:
                self.last_direction = -1.0
                self.last_speed = 5.0
                self.get_logger().info('reach left side')
                
            elif self.lidar_left - self.lidar_right < -0.045:
                self.last_direction = 1.0
                self.last_speed = 5.0
                self.get_logger().info('reach rigth side')
        
        #Letzte bewegung wird fortgeführt
        if self.last_direction is not None:
            if self.last_speed is None:
                    self.last_speed = 5.0

            if self.last_direction == -1.0:
                self.controller.right(percent=self.last_speed)
                self.get_logger().debug(f'rechts {self.last_speed}')
                return

            elif self.last_direction == 1.0:
                self.controller.left(percent=self.last_speed)
                self.get_logger().debug(f'links {self.last_speed}')
                return
        else:
            #Wenn noch nichts gefunden wurde fährt der Roboter nach links
            self.get_logger().debug('no qr found driving left')
            self.last_direction = 1.0
            self.last_speed = 5.0


    def final_centering(self,offset):

        if offset < 0.02 + self.goal_offset and offset > -0.02 + self.goal_offset:
            self.controller.stop()
                
            msg_dock_feedback = DockFeedback()
            msg_dock_feedback.time = self.dock_time
            msg_dock_feedback.process = 'qr_center'
            msg_dock_feedback.success = True
            self.publisher_node_state.publish(msg=msg_dock_feedback)
            self.stop_node()
            return None,None
            
        elif offset > 0.02 + self.goal_offset:
            if offset > 0.3 + self.goal_offset:
                self.controller.left(percent=5.0)
                self.get_logger().debug('links')
                return 1.0,5.0
            else:
                self.controller.left(percent=1.0)
                self.get_logger().debug('links langsam')
                return 1.0,1.0

        else:
            if offset < -0.3 +self.goal_offset:
                self.controller.right(percent=5.0)
                self.get_logger().debug('rechts')
                return -1.0,5.0
            else:
                self.controller.right(percent=1.0)
                self.get_logger().debug('rechts langsam')
                return -1.0,1.0


def main(args=None):
    rclpy.init(args=args)

    center_qr = CenterQr()
    rclpy.spin(center_qr)
    center_qr.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()