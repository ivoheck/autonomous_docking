#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

from autonomous_docking_pkg import motor_controller
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile


class DriveToQr(Node):
    def __init__(self):
        super().__init__('drive_to_qr')
        self.tollerace = 0.05 # wert innerhalg geradeaus gefahren wird
        self.lidar_msg = None

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/qr_pos',
            self.listener_callback,
            10)
        
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.foundQr = False
        self.controller = motor_controller.MotorControllerHelper()

    #TODO: das vieleicht in einen service packen
    def get_lidar(self):
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.lidar_subscription

    def lidar_callback(self,msg):
        self.lidar_msg = msg
        self.destroy_subscription(self.lidar_subscription)
        
    def listener_callback(self, msg):
        self.get_lidar()
        #TODO: das muss besser gemacht werden evtl mit nem service oder so
        if self.lidar_msg is not None:
            front_dist = self.lidar_msg.ranges[0]
            if front_dist <= 0.3:
                self.controller.stop()
                self.get_logger().info('Facing wall')
                rclpy.shutdown()
                return

        direction,dif = msg.data[0],msg.data[1]
        if direction != 0.0:
            self.foundQr = True
            if dif <= self.tollerace:
                self.controller.front(20.0)

            elif dif > self.tollerace:
                self.controller.drive_curve(direction=direction,percent_x=20.0,percent_z=3.0)

        else:
            if self.foundQr and self.lidar_msg is not None:

                #Der roboter fährt auch ohne QR code so lange bis die distanz nach vorne zu klein wird
                if front_dist <= 0.3:
                    self.controller.stop()
                    self.get_logger().info('Facing wall')
                    rclpy.shutdown()

            else:
                #Wenn nie ein QR code gefunden wurde
                self.controller.stop()


def main(args=None):
    rclpy.init(args=args)

    drive_to_qr = DriveToQr()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(drive_to_qr)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            drive_to_qr.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()