#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

from autonomous_docking_pkg import motor_controller
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Undock(Node):

    def __init__(self):
        super().__init__('undock')
        self.controller = motor_controller.MotorControllerHelper()
        self.publisher_lock = self.create_publisher(Bool, '/lock', 10)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription  # prevent unused variable warning
        self.lidar_front = None
        self.open_lock()
        self.timer = self.create_timer(0.1, self.start_undock)

    def start_undock(self):
        if self.lidar_front is not None and self.lidar_front < 0.5:
            self.controller.back(10.0)

        elif self.lidar_front is None:
            self.controller.stop()

        else:
            self.controller.stop()
            self.close_lock()
            rclpy.shutdown()

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