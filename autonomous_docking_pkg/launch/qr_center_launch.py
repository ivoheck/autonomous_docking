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
    qr_center_scan_node = Node(
            package='autonomous_docking_pkg',
            executable='qr_center_scan_node',
        )
    qr_center_node = Node(
            package='autonomous_docking_pkg',
            executable='qr_center_node',
        )
    return LaunchDescription([
        qr_center_scan_node,
        qr_center_node,
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=qr_center_node,
                on_completion=[
                    EmitEvent(event=Shutdown())
                ]
            )
        ),
    ])