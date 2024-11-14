#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-Event-Handlers.html
#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch import LaunchDescription
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

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
    
    face_wall_node = Node(
            package='autonomous_docking_pkg',
            executable='face_wall_node',
        )
    
    drive_to_wall_node = Node(
            package='autonomous_docking_pkg',
            executable='drive_to_wall_node',
        )

    qr_center_node = Node(
            package='autonomous_docking_pkg',
            executable='qr_center_node',
        )
    
    probe_docking_node = Node(
            package='autonomous_docking_pkg',
            executable='probe_docking_node',
        )
    
    probe_docking_service_node = Node(
            package='autonomous_docking_pkg',
            executable='probe_docking_service_node',
        )

    return LaunchDescription([
        ws_nr_launch_arg,
        qr_scan_node,
        find_qr_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=find_qr_node,
                on_exit=[
                    drive_to_qr_node,
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=drive_to_qr_node,
                on_exit=[
                    face_wall_node,
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=face_wall_node,
                on_exit=[
                    drive_to_wall_node,
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=drive_to_wall_node,
                on_exit=[
                    qr_center_node,
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=qr_center_node,
                on_exit=[
                    probe_docking_service_node,
                    probe_docking_node,
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=probe_docking_node,
                on_exit=[
                    EmitEvent(event=Shutdown()),
                ]
            )
        ),

    ])
