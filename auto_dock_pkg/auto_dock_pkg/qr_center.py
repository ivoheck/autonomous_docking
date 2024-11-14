#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

from autonomous_docking_pkg import motor_controller
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
        self.last_diff = None
        self.subscription = self.create_subscription(
            QrPos,
            '/qr_pos',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback_lidar,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription_lidar  # prevent unused variable warning

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

    def listener_callback_manage_node_state(self, msg):
        if msg.trigger:
            self.start_node()
            self.dock_time = msg.time
        else:
            self.stop_node()

    def start_node(self):
        #self.get_logger().info('start qr_center_node')
        self.node_state = True
        self.time_start = time.time()

    def stop_node(self):
        #self.get_logger().info('stop qr_center_node')
        self.node_state = False

    def listener_callback_lidar(self, msg):
        if self.node_state == False:
            return

        if time.time() - self.time_start > 60.0:
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

        if msg.ranges[11] != inf:
            self.lidar_left = msg.ranges[11]

        if msg.ranges[709] != inf:
            self.lidar_right = msg.ranges[709]
        
    def listener_callback(self, msg):
        if self.node_state == False:
            return
        
        if self.lidar_front is not None and self.lidar_front < 0.25:
            self.controller.back(percent=5.0)
            return
        
        elif self.lidar_front is not None and self.lidar_front > 0.3:
            self.controller.front(percent=5.0)
            return

        direction,diff,qrcode = msg.direction,msg.offset,msg.qrcode
        if direction != 0.0:
            self.last_direction = direction
            self.last_diff = diff
            self.last_qrcode = qrcode
            self.drive(direction=direction,diff=diff,qrcode=qrcode)
        else:
            if self.last_direction is not None and self.last_diff is not None and self.last_qrcode is not None:
                self.drive(direction=self.last_direction,diff=self.last_diff,qrcode=self.last_qrcode)

            else:
                #Qr-code wurde noch nie gesehen
                if self.lidar_front is None or self.lidar_left is None:
                    return
                
                #Wenn der Roboter zu weit links ist muss das zentrum rechts sein
                if self.lidar_left - self.lidar_front > 0.045:
                    self.last_direction = -1.0
                
                else:
                    #Wenn noch nichts gefunden wurde fährt der Roboter nach links
                    self.drive(direction=1.0)

    def drive(self,direction,diff,qrcode):

        if qrcode == 'l':
            self.controller.right(percent=5.0)

        elif qrcode == 'r':
            self.controller.left(percent=5.0)

        elif qrcode == 'c':
            if diff < 0.02 + self.goal_offset and diff > -0.02 + self.goal_offset:
                #print('end')
                self.controller.stop()
                
                msg_dock_feedback = DockFeedback()
                msg_dock_feedback.time = self.dock_time
                msg_dock_feedback.process = 'qr_center'
                msg_dock_feedback.success = True
            
                self.publisher_node_state.publish(msg=msg_dock_feedback)
                self.stop_node()
                return
            
            elif diff > 0.02 + self.goal_offset:
                self.controller.left(percent=5.0)

            else:
                self.controller.right(percent=5.0)

        elif direction == -1.0:
            self.controller.right(percent=5.0)

        elif direction == 1.0:
            self.controller.left(percent=5.0)


def main(args=None):
    rclpy.init(args=args)

    center_qr = CenterQr()

    rclpy.spin(center_qr)
    
    center_qr.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()