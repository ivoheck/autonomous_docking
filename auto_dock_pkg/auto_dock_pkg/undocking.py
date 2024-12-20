#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

from robot_controll_pkg import motor_controller
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String

class Undock(Node):

    def __init__(self):
        super().__init__('undock')
        self.controller = motor_controller.MotorControllerHelper()
        self.publisher_lock = self.create_publisher(Bool, '/lock', 1)
        self.publisher_current_state = self.create_publisher(String, '/current_state', 1)
        self.publisher_current_ws = self.create_publisher(String, '/current_ws', 1)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription  
        self.lidar_front = None

        self.subscription_start_undocking = self.create_subscription(
            Bool,
            '/start_undock',
            self.listener_callback_start_undocking,
            1)
        self.subscription_start_undocking  

    def listener_callback_start_undocking(self,msg):
        if msg.data:
            self.open_lock()
            self.timer = self.create_timer(0.1, self.start_undock)

    def start_undock(self):
        msg = String()
        msg.data = 'undocking'
        self.publisher_current_state.publish(msg)
        
        if self.lidar_front is not None and self.lidar_front < 0.5:
            self.controller.back(10.0)

        elif self.lidar_front is None:
            self.controller.stop()

        else:
            self.controller.stop()
            self.close_lock()

            self.get_logger().info('undocking success')

            msg_state = String()
            msg_state.data = 'waiting'
            self.publisher_current_state.publish(msg_state)

            msg_ws = String()
            msg_ws.data = 'no_ws'
            self.publisher_current_ws.publish(msg_ws)

            self.timer.destroy()

    def listener_callback(self,msg):
        self.lidar_front = msg.ranges[0]

    def open_lock(self):
        msg = Bool()
        msg.data = True
        self.publisher_lock.publish(msg)

    def close_lock(self):
        msg = Bool()
        msg.data = False
        self.publisher_lock.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    undock = Undock()
    rclpy.spin(undock)
    
    undock.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()