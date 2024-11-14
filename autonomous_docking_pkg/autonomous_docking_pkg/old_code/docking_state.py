import os
import rclpy
import launch
from rclpy.node import Node
from std_msgs.msg import String

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from custom_interfaces.msg import DockTrigger

class DockingState(Node):
    def __init__(self):
        super().__init__('docking_state')
        self.subscription = self.create_subscription(
            DockTrigger,
            '/start_dock',
            self.listener_callback_start_dock,
            1)
        self.subscription

        self.subscription_state = self.create_subscription(
            String,
            '/current_state',
            self.listener_callback_state,
            1)
        self.subscription_state

        self.current_dock = None
        self.current_dock_state = None

    def listener_callback_start_dock(self, msg):
        self.current_dock = msg.time
        self.current_dock_state = True

    def listener_callback_state(self, msg):
        if msg.data == 'docking_fail':
            self.current_dock_state = False
  
        

def main(args=None):
    rclpy.init(args=args)

    docking_state = DockingState()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(docking_state)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            executor.shutdown()
            docking_state.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()