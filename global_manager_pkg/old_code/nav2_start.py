import os
import rclpy
import launch
from rclpy.node import Node
from std_msgs.msg import Bool

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

class StartNavigation(Node):

    def __init__(self):
        super().__init__('start_navigation')
        self.subscription = self.create_subscription(
            Bool,
            '/start_nav2',
            self.listener_callback,
            1)
        self.subscription

        #LLM
        self.launch_service = launch.LaunchService()
        
    def listener_callback(self,msg):
        if msg.data:
            package_name = 'nav2_bringup'
            launch_file_dir = os.path.join(get_package_share_directory(package_name), 'launch')


            launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/navigation_launch.py'])
            )

            self.launch_service.include_launch_description(launch_description)
            self.launch_service.run()

        else:
            if self.launch_service.is_running():
                self.launch_service.shutdown()
            #LLM

def main(args=None):
    rclpy.init(args=args)

    start_navigation = StartNavigation()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(start_navigation)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            start_navigation.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()