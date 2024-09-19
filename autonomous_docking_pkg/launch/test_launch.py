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
    first_node = Node(
            package='autonomous_docking_pkg',
            executable='my_node',
        )
    second_node = Node(
            package='autonomous_docking_pkg',
            executable='my_node2',
        )

    next_node = Node(
            package='autonomous_docking_pkg',
            executable='my_node3',
        )
    return LaunchDescription([
        first_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=first_node,
                on_exit=[
                    second_node,
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=second_node,
                on_exit=[
                    next_node,
                ]
            )
        ),
    ])