#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-Event-Handlers.html
#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

def generate_launch_description():
    ws_nr = LaunchConfiguration('ws_nr')

    ws_nr_launch_arg = DeclareLaunchArgument(
        'ws_nr',
        default_value='ws_1'
        )
    
    qr_scan_node = Node(
            package='autonomous_docking_pkg',
            executable='qr_scan_node',
            parameters=[{'ws_nr': ws_nr}]
        )

    find_qr_node = Node(
            package='autonomous_docking_pkg',
            executable='find_qr_node',
        )
    
    drive_to_qr_node = Node(
            package='autonomous_docking_pkg',
            executable='drive_to_qr_node',
        )
    
    return LaunchDescription([
        qr_scan_node,
        find_qr_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=find_qr_node,
                on_exit=[
                    handle_node_exit(0,drive_to_qr_node),
                ]
            )
        ),
    ])


def handle_node_exit(returncode,node):
    if returncode == 0:
        node