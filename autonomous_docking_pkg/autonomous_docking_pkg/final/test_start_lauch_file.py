import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import subprocess


class StartLaunch(Node):

    def __init__(self):
        super().__init__('start_launch')
        self.start()

    def start(self):
        subprocess.run(["ros2", "launch", "autonomous_docking_pkg", "autonomous_docking_launch.py"])


def main(args=None):
    rclpy.init(args=args)

    start_launch = StartLaunch()

    rclpy.spin(start_launch)

    start_launch.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()