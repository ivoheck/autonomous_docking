#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

from autonomous_docking_pkg import motor_controller
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from custom_interfaces.msg import QrPos
from std_msgs.msg import String
import time
from custom_interfaces.msg import DockTrigger
from custom_interfaces.msg import DockFeedback
from std_msgs.msg import Bool

class FindQr(Node):

    def __init__(self):
        super().__init__('find_qr')
    
        self.controller = motor_controller.MotorControllerHelper()
        self.publisher_current_state = self.create_publisher(String, '/current_state', 1)
        self.publisher_node_state = self.create_publisher(DockFeedback, '/dock_feedback', 1)

        self.subscription = self.create_subscription(
            DockTrigger,
            'trigger_dock_node/find_qr_node',
            self.listener_callback_manage_node_state,
            1)
        self.subscription

        self.state = True
        self.time_state = True
        self.time_start = None
        self.qr_state = False
        self.last_direction = None

        self.dock_time = None

        self.node_state = False

    def listener_callback_manage_node_state(self, msg):
        if msg.trigger:
            self.start_node()
            self.dock_time = msg.time
        else:
            self.stop_node()

    def start_node(self):
        #self.get_logger().info('start find_qr_node')
        self.subscription = self.create_subscription(
            QrPos,
            '/qr_pos',
            self.listener_callback,
            10)
        self.subscription

        self.state = True
        self.time_start = time.time()
        self.qr_state = False
        self.last_direction = None

        self.node_state = True

    def stop_node(self):
        #self.get_logger().info('stop find_qr_node')
        self.subscription = None

        self.state = True
        self.time_state = True
        self.time_start = None
        self.qr_state = False
        self.last_direction = None

        self.node_state = False
        
    def listener_callback(self, msg):
        if self.node_state == False:
            return

        if time.time() - self.time_start > 60.0 and self.qr_state == False:#60
            self.controller.stop()
            #self.get_logger().error('QR not Found in time (60s)')

            msg_dock_feedback = DockFeedback()
            msg_dock_feedback.time = self.dock_time
            msg_dock_feedback.process = 'find_qr'
            msg_dock_feedback.success = False
            msg_dock_feedback.reason = 'time_out'
            
            self.publisher_node_state.publish(msg=msg_dock_feedback)
            self.stop_node()
            return

        if time.time() - self.time_start > 80.0 and self.qr_state == True:
            self.controller.stop()
            #self.get_logger().error('QR Found But Failed alignment time (80s)')

            msg_dock_feedback = DockFeedback()
            msg_dock_feedback.time = self.dock_time
            msg_dock_feedback.process = 'find_qr'
            msg_dock_feedback.success = False
            msg_dock_feedback.reason = 'time_out'
            
            self.publisher_node_state.publish(msg=msg_dock_feedback)
            self.stop_node()
            return
    
        if msg.direction == 1.0:
            self.qr_state = True
            self.last_direction = 1.0
            if abs(msg.offset) < 0.1:
                self.controller.stop()
                self.get_logger().info('QR Found')
                
                msg_dock_feedback = DockFeedback()
                msg_dock_feedback.time = self.dock_time
                msg_dock_feedback.process = 'find_qr'
                msg_dock_feedback.success = True
            
                self.publisher_node_state.publish(msg=msg_dock_feedback)
                self.stop_node()

            elif abs(msg.offset) < 0.6:
                self.controller.turn_left(5.0)
            else:
                self.controller.turn_left(10.0)

        elif msg.direction == -1.0:
            self.qr_state = True
            self.last_direction = -1.0
            if abs(msg.offset) < 0.1:
                self.controller.stop()
                self.get_logger().info('QR Found')
                
                msg_dock_feedback = DockFeedback()
                msg_dock_feedback.time = self.dock_time
                msg_dock_feedback.process = 'find_qr'
                msg_dock_feedback.success = True
            
                self.publisher_node_state.publish(msg=msg_dock_feedback)
                self.stop_node()

            elif abs(msg.offset) < 0.6:
                self.controller.turn_right(5.0)
            else:
                self.controller.turn_right(10.0)

        #das wird gemacht da manchmal zwischendurch kein qr code gescant wird
        elif self.last_direction == 1.0:
            self.controller.turn_left(5.0)

        elif self.last_direction == -1.0:
            self.controller.turn_right(5.0)

        else:
            self.controller.turn_left(20.0)
    

def main(args=None):
    rclpy.init(args=args)

    find_qr = FindQr()

    rclpy.spin(find_qr)
    
    find_qr.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()