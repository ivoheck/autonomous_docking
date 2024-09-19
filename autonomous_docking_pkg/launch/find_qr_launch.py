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

def generate_launch_description():
    find_qr_node = Node(
            package='autonomous_docking_pkg',
            executable='find_qr_node',
        )
    
    scan_find_qr_node = Node(
            package='autonomous_docking_pkg',
            executable='scan_find_qr_node',
        )
    return LaunchDescription([
        find_qr_node,
        scan_find_qr_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=find_qr_node,
                on_exit=[
                    EmitEvent(event=Shutdown())
                ]
            )
        ),
    ])