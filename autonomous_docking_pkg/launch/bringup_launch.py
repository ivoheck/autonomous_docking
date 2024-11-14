#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-Event-Handlers.html
#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    motor_controller_node = Node(
            package='autonomous_docking_pkg',
            executable='motor_controller_node',
        )

    odometry_publisher_node = Node(
        package='autonomous_docking_pkg',
        executable='odometry_publisher_node',
    )

    lock_controller_node = Node(
            package='autonomous_docking_pkg',
            executable='lock_controller_node',
        )
    
    lidar_fillter_node = Node(
            package='autonomous_docking_pkg',
            executable='lidar_fillter_node'
    )

    undocking_node = Node(
        package='autonomous_docking_pkg',
        executable='undocking_node'
    )
    
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('sllidar_ros2'), '/launch/sllidar_c1_launch.py'])
    )

    cube_bot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('cube_bot_description'), '/launch/display.launch.py'])
    )

    docking_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('auto_dock_pkg'), '/launch/auto_dock_bringup_launch.py'])
    )

    return LaunchDescription([
        motor_controller_node,
        cube_bot_description_launch,
        odometry_publisher_node,
        sllidar_launch,
        lidar_fillter_node,
        lock_controller_node,
        undocking_node,
        docking_bringup_launch
    ])