#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-Event-Handlers.html
#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    find_qr_node = Node(
       package='auto_dock_pkg',
       executable='find_qr_node'
    )

    drive_to_qr_node = Node(
       package='auto_dock_pkg',
       executable='drive_to_qr_node'
    )

    qr_scan_node = Node(
       package='auto_dock_pkg',
       executable='qr_scan_node'
    )

    face_wall_node = Node(
       package='auto_dock_pkg',
       executable='face_wall_node'
    )

    drive_to_wall_node = Node(
       package='auto_dock_pkg',
       executable='drive_to_wall_node'
    )

    qr_center_node = Node(
      package='auto_dock_pkg',
       executable='qr_center_node' 
    )

    final_docking_node = Node(
      package='auto_dock_pkg',
       executable='final_docking_node' 
    )

    docking_manager_node = Node(
      package='auto_dock_pkg',
       executable='docking_manager_node' 
    )
   
    return LaunchDescription([
        find_qr_node,
        drive_to_qr_node,
        qr_scan_node,
        face_wall_node,
        drive_to_wall_node,
        qr_center_node,
        final_docking_node,
        docking_manager_node,
    ])