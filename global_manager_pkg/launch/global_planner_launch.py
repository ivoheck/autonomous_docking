#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html
#https://roboticsbackend.com/ros2-yaml-params/

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('global_manager_pkg'),
        'config',
        'workplace.yaml'
        )
    
    global_planner_node = Node(
       package='global_manager_pkg',
       executable='global_planner_node',
       parameters = [config] 
    )

    return LaunchDescription([
        global_planner_node,
    ])