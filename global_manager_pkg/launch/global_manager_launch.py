#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

from launch_ros.actions import Node

from launch import LaunchDescription

def generate_launch_description():
    
    manager_node = Node(
       package='global_manager_pkg',
       executable='manager_node', 
    )

    return LaunchDescription([
        manager_node
    ])