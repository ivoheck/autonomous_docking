#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-Event-Handlers.html
#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

from launch_ros.actions import Node

from launch import LaunchDescription

def generate_launch_description():
    undock_node = Node(
            package='autonomous_docking_pkg',
            executable='undock_node',
        )
    
    return LaunchDescription([
        undock_node,
    ])