#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

from autonomous_docking_pkg import motor_controller
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String
from custom_interfaces.msg import QrPos
import time

class Undock(Node):

    def __init__(self):
        super().__init__('undock')
        self.controller = motor_controller.MotorControllerHelper()
        self.publisher_lock = self.create_publisher(Bool, '/lock', 1)
        self.publisher_current_state = self.create_publisher(String, '/current_state', 1)
        self.publisher_current_ws = self.create_publisher(String, '/current_ws', 1)

        self.status = True

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription  # prevent unused variable warning
        self.lidar_front = None

        self.subscription_qr_pos = self.create_subscription(
            QrPos,
            '/qr_pos',
            self.listener_callback_qr_pos,
            1)
        
        self.subscription_qr_pos
        self.qr_pos = None
        self.qr_pos_final = None

        self.subscription_start_undocking = self.create_subscription(
            Bool,
            '/start_undock',
            self.listener_callback_start_undocking,
            1)
        self.subscription_start_undocking  # prevent unused variable warning

        #Zum manulellen undocking
        if __name__ == '__main__':
            self.open_lock()
            self.timer = self.create_timer(0.1, self.start_undock)

    def listener_callback_start_undocking(self,msg):
        if msg.data:
            self.open_lock()
            self.timer = self.create_timer(0.1, self.start_undock)

    def start_undock(self):
        msg = String()
        msg.data = 'undocking'
        self.publisher_current_state.publish(msg)

        if self.lidar_front is not None and self.lidar_front < 0.25:
            self.controller.back(5.0)
        
        elif self.lidar_front is not None and self.lidar_front < 0.5:
            if self.status:
                self.status = False
                #self.controller.stop()
                #time.sleep(1)
                self.qr_pos_final = self.qr_pos
                print(self.qr_pos_final)

            self.controller.back(10.0)

        elif self.lidar_front is None:
            self.controller.stop()

        else:
            self.controller.stop()
            self.close_lock()

            msg_state = String()
            msg_state.data = 'undocked'
            self.publisher_current_state.publish(msg_state)

            msg_ws = String()
            msg_ws.data = 'no_ws'
            self.publisher_current_ws.publish(msg_ws)

            self.timer.destroy()
            #rclpy.shutdown()

    def listener_callback(self,msg):
        self.lidar_front = msg.ranges[0]

    def listener_callback_qr_pos(self, msg):
        self.qr_pos = msg

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

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(undock)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            undock.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()