#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-Event-Handlers.html
#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

from launch_ros.actions import Node

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    param_file = os.path.join( '/home/ivo/ros2_ws/src/global_manager_pkg/config', 'mapper_params_online_async.yaml')
    
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        #output='screen',
        parameters=[param_file]
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('nav2_bringup'), '/launch/navigation_launch.py'])
    )

    nav2_start_node = Node(
        package='global_manager_pkg',
        executable='nav2_start_node',
    )

    path_follower_node = Node(
        package='global_manager_pkg',
        executable='path_follower_node', 
    )

    return LaunchDescription([
        slam_toolbox,
        #nav2_start_node,
        nav2_bringup_launch,
        #path_follower_node,
    ])