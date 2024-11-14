#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

from autonomous_docking_pkg import motor_controller
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from custom_interfaces.msg import QrPos
from std_msgs.msg import String
import time
from custom_interfaces.msg import DockTrigger

#TODO: oziliert wenn zu weit weg 
class FindQr(Node):

    def __init__(self):
        super().__init__('find_qr')
        self.subscription = self.create_subscription(
            QrPos,
            '/qr_pos',
            self.listener_callback,
            10)
        self.state = True
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.controller = motor_controller.MotorControllerHelper()

        self.time_state = True
        self.time_start = None
        self.qr_state = False

        self.last_direction = None

        self.publisher_current_state = self.create_publisher(String, '/current_state', 1)
        self.publisher_start_docking = self.create_publisher(DockTrigger, '/start_dock', 1)

        #msg_state = String()
        #msg_state.data = 'docking'
        #self.get_logger().info('Loocking for WS')
        #self.publisher_current_state.publish(msg_state)
        
    def listener_callback(self, msg):
        if self.time_state:
            self.time_start= time.time()
            self.time_state = False

        if time.time() - self.time_start > 60.0 and self.qr_state == False:#60
            self.controller.stop()
            self.get_logger().info('QR not Found in time (60s)')

            msg = String()
            msg.data = 'docking_fail'
            self.publisher_current_state.publish(msg)
            rclpy.shutdown()

        if time.time() - self.time_start > 80.0 and self.qr_state == True:
            self.controller.stop()
            self.get_logger().info('QR Found But Failed alignment time (80s)')

            msg = String()
            msg.data = 'docking_fail'
            self.publisher_current_state.publish(msg)

            rclpy.shutdown()
    
        if msg.direction == 1.0:
            self.qr_state = True
            self.last_direction = 1.0
            if abs(msg.offset) < 0.1:
                self.controller.stop()
                self.get_logger().info('QR Found')
                rclpy.shutdown()

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
                rclpy.shutdown()

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

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(find_qr)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            find_qr.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()