import os
import rclpy
import launch
from rclpy.node import Node
from std_msgs.msg import Bool

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

class StartDocking(Node):

    def __init__(self):
        super().__init__('start_docking')
        self.subscription = self.create_subscription(
            Bool,
            '/start_dock',
            self.listener_callback,
            1)
        self.subscription
        
    def listener_callback(self,msg):
        if msg.data:
            #LLM
            package_name = 'autonomous_docking_pkg'
            launch_file_dir = os.path.join(get_package_share_directory(package_name), 'launch')

            # Erstellen einer Launch-Beschreibung
            launch_service = launch.LaunchService()
            launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/autonomous_docking_launch.py'])
            )

            # Starten der Launch-Datei
            launch_service.include_launch_description(launch_description)
            launch_service.run()
            self.get_logger().info('Start docking node')
            #LLM


def main(args=None):
    rclpy.init(args=args)

    start_docking = StartDocking()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(start_docking)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            start_docking.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()