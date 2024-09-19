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
    dock_node = Node(
            package='autonomous_docking_pkg',
            executable='dock_node',
        )
    return LaunchDescription([
        dock_node,
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=dock_node,
                on_completion=[
                    EmitEvent(event=Shutdown())
                ]
            )
        ),
    ])