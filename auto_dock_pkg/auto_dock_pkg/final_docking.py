import sys
import json

from robot_controll_pkg import motor_controller
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from custom_interfaces.msg import DockTrigger
from custom_interfaces.msg import DockFeedback
import time

class FinalDocking(Node):
    def __init__(self):
        super().__init__('final_dock')
        
        self.publisher_lock = self.create_publisher(Bool, '/lock', 1)
        self.publisher_current_state = self.create_publisher(String, '/current_state', 1)
        self.publisher_start_docking = self.create_publisher(DockTrigger, '/start_dock', 1)
        self.publisher_node_state = self.create_publisher(DockFeedback, '/dock_feedback', 1)
        self.controller = motor_controller.MotorControllerHelper()

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            )
        
        self.subscription_node_state = self.create_subscription(
            DockTrigger,
            'dock_trigger/final_docking_trigger',
            self.listener_callback_manage_node_state,
            1)
        self.subscription_node_state
        
        self.lidar_subscription

        self.lidar_ranges = None
        self.lidar_list = []
        self.check_value_len = 25
        self.check_value_dis = 0.001
        self.dock_time = None
        self.node_state = False

    def listener_callback_manage_node_state(self, msg):
        if msg.trigger:
            self.start_node()
            self.dock_time = msg.time
        else:
            self.stop_node()

    def start_node(self):
        self.node_state = True

        self.lidar_ranges = None
        self.lidar_list = []
        self.check_value_len = 25
        self.check_value_dis = 0.001
        self.dock_time = None
        
        self.time_start = time.time()
        self.timer = self.create_timer(0.1, self.docking)

    def stop_node(self):
        self.node_state = False

        self.lidar_ranges = None
        self.lidar_list = []
        self.check_value_len = 25
        self.check_value_dis = 0.001
        self.dock_time = None

        try:
            self.timer.destroy()
        except:
            pass

    def open_lock(self):
        msg = Bool()
        msg.data = True
        self.publisher_lock.publish(msg)

    def close_lock(self):
        msg = Bool()
        msg.data = False
        self.publisher_lock.publish(msg)

    def lidar_callback(self,msg):
        if self.node_state == False:
            return
        
        self.lidar_ranges = msg.ranges

    def docking(self):
        if time.time() - self.time_start > 20.0:#20
            self.controller.stop()

            msg_dock_feedback = DockFeedback()
            msg_dock_feedback.time = self.dock_time
            msg_dock_feedback.process = 'final_docking'
            msg_dock_feedback.success = False
            msg_dock_feedback.reason = 'time_out'
            
            self.publisher_node_state.publish(msg=msg_dock_feedback)
            self.stop_node()
            return
        
        lidar_front = self.lidar_ranges[0]

        if lidar_front is None:
            return
        
        self.get_logger().debug(str(lidar_front))

        self.lidar_list.append(lidar_front)

        if len(self.lidar_list) > self.check_value_len:
            self.lidar_list.pop(0)

            #self.get_logger().info(str(abs(self.lidar_list[0] - self.lidar_list[self.check_value_len -1]) ))
            if abs(self.lidar_list[0] - self.lidar_list[self.check_value_len -1]) < self.check_value_dis:
                self.close_lock()
                self.get_logger().info('docking fail')
                self.controller.stop()
  
                msg_dock_feedback = DockFeedback()
                msg_dock_feedback.time = self.dock_time
                msg_dock_feedback.process = 'final_docking'
                msg_dock_feedback.success = False
            
                self.publisher_node_state.publish(msg=msg_dock_feedback)
                self.stop_node()
                return

        if (lidar_front >= 0.185 or lidar_front == 'inf'):
            self.controller.front(percent=10.0)

        elif lidar_front >= 0.175: 
            self.open_lock()
            self.controller.front(percent=30.0)

        elif lidar_front >= 0.164: 
            self.controller.front(percent=100.0)

        #Docking erfolgreich
        elif lidar_front < 0.164:
            self.get_logger().info('docking success')
            self.close_lock()
            self.controller.stop()

            msg_dock_feedback = DockFeedback()
            msg_dock_feedback.time = self.dock_time
            msg_dock_feedback.process = 'final_docking'
            msg_dock_feedback.success = True
            
            self.publisher_node_state.publish(msg=msg_dock_feedback)
            self.stop_node()
            return



def main(args=None):
    rclpy.init(args=args)

    final_dock = FinalDocking()

    rclpy.spin(final_dock)
    
    final_dock.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()