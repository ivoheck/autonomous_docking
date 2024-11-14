#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

#subscriber KLasse https://docs.ros.org/en/kinetic/api/rospy/html/rospy.topics.Subscriber-class.html


from autonomous_docking_pkg import motor_controller
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile


class DriveToWall(Node):

    def __init__(self):
        super().__init__('drive_to_wall')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription  # prevent unused variable warning
        self.controller = motor_controller.MotorControllerHelper()
        
    def listener_callback(self, msg):
        front = msg.ranges[0]
        print(front)
        
        if front >= 0.3:
            self.controller.front(percent=10.0)

        elif front <= 0.25:
            self.controller.back(percent=5.0)
        
        else:
            self.controller.stop()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    drive_to_wall = DriveToWall()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(drive_to_wall)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            drive_to_wall.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()