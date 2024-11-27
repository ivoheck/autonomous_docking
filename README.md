# autonomous_docking

## Basic Usage
### Start Bringup
`ros2 launch robot_controll_pkg bringup_launch.py`

### Start docking to Workspace 1
`ros2 topic pub /start_dock custom_interfaces/DockTrigger "{trigger: true, wsnumber: 1}" --once`

### Start navigation to Workspace 1 and dock subsequently
`ros2 topic pub /dock_goal std_msgs/msg/String "data: 'ws_1'" --once`

### Undock from Current Workspace
`ros2 topic pub /start_undock std_msgs/msg/Bool "data: true" --once`
