# autonomous_docking

`ros2 topic pub /start_dock custom_interfaces/DockTrigger "{trigger: true, wsnumber: 1}" --once`

`ros2 topic pub /dock_goal std_msgs/msg/String "data: 'ws_1'"`
