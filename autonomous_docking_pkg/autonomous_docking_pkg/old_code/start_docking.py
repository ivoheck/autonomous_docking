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

class StartDocking(Node):
    def __init__(self):
        super().__init__('start_docking')
        self.subscription = self.create_subscription(
            DockTrigger,
            '/start_dock',
            self.listener_callback,
            1)
        self.subscription

        self.publisher_current_state = self.create_publisher(String, '/current_state', 1)
        self.publisher_current_ws = self.create_publisher(String, '/current_ws', 1)

        #LLM
        package_name = 'autonomous_docking_pkg'
        self.launch_file_dir = os.path.join(get_package_share_directory(package_name), 'launch')
        
    def listener_callback(self,msg):
        self.launch_service = launch.LaunchService()

        if msg.trigger:
            #Ws argument wird an launch file übergeben
            self.launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([self.launch_file_dir, '/autonomous_docking_launch.py']),
            launch_arguments={
                    'ws_nr': f'ws_{msg.wsnumber}' 
                }.items()
            )
            self.launch_service.include_launch_description(self.launch_description)
            #LLM

            self.get_logger().info(f'Start docking node ws: {msg.wsnumber}')

            msg_ws = String()
            msg_ws.data = f'{msg.wsnumber}'
            self.publisher_current_ws.publish(msg_ws)

            #LLM
            self.launch_service.run()
            
        elif msg.trigger == False:
            self.launch_service.shutdown()
            #LLM
            self.get_logger().info(f'Docking launch Shutdown')
        

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
        if rclpy.ok():
            executor.shutdown()
            start_docking.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()