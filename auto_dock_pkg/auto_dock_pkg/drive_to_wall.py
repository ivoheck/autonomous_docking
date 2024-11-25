#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

#subscriber KLasse https://docs.ros.org/en/kinetic/api/rospy/html/rospy.topics.Subscriber-class.html


from robot_controll_pkg import motor_controller
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from custom_interfaces.msg import DockTrigger
from custom_interfaces.msg import DockFeedback
import time


class DriveToWall(Node):

    def __init__(self):
        super().__init__('drive_to_wall')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription 

        self.subscription_node_state = self.create_subscription(
            DockTrigger,
            'trigger_dock_node/drive_to_wall_node',
            self.listener_callback_manage_node_state,
            1)
        self.subscription_node_state

        self.controller = motor_controller.MotorControllerHelper()
        self.publisher_node_state = self.create_publisher(DockFeedback, '/dock_feedback', 1)

        self.dock_time = None
        self.node_state = False

    def listener_callback_manage_node_state(self, msg):
        if msg.trigger:
            self.start_node()
            self.dock_time = msg.time
        else:
            self.stop_node()

    def start_node(self):
        #self.get_logger().info('start drive_to_wall_node')
        self.node_state = True
        self.time_start = time.time()

    def stop_node(self):
        #self.get_logger().info('stop drive_to_wall_node')
        self.node_state = False
        
    def listener_callback(self, msg):
        if self.node_state == False:
            return

        if time.time() - self.time_start > 10.0:
            self.controller.stop()

            msg_dock_feedback = DockFeedback()
            msg_dock_feedback.time = self.dock_time
            msg_dock_feedback.process = 'drive_to_wall'
            msg_dock_feedback.success = False
            msg_dock_feedback.reason = 'time_out'
            
            self.publisher_node_state.publish(msg=msg_dock_feedback)
            self.stop_node()
            return

        front = msg.ranges[0]
        print(front)
        
        if front >= 0.25:
            self.controller.front(percent=10.0)

        elif front <= 0.20:
            self.controller.back(percent=5.0)
        
        else:
            self.controller.stop()

            msg_dock_feedback = DockFeedback()
            msg_dock_feedback.time = self.dock_time
            msg_dock_feedback.process = 'drive_to_wall'
            msg_dock_feedback.success = True
            
            self.publisher_node_state.publish(msg=msg_dock_feedback)
            self.stop_node()

def main(args=None):
    rclpy.init(args=args)

    drive_to_wall = DriveToWall()

    rclpy.spin(drive_to_wall)
    
    drive_to_wall.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()