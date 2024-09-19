import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import time

import motor_controller


class KeyboardPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.w = 0.0
        self.a = 0.0
        self.d = 0.0
        self.q = 0.0
        self.e = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.controller = motor_controller.MotorControllerHelper()
        self.start_loop()

    def print_stuff(self):
        print('x',self.x)
        print('y',self.y)
        print('z',self.z)

    def start_loop(self):
        while True:
            inp = str(input('key: '))

            if inp == 'w':
                self.w += 10.0
                self.x = 0.0
                self.controller.front(self.w)
                self.print_stuff()

            elif inp == 'a':
                self.a += 10.0
                self.d = 0.0
                self.controller.left(self.a)
                self.print_stuff()

            elif inp == 's':
                self.w = 0.0
                self.a = 0.0
                self.d = 0.0
                self.q = 0.0
                self.e = 0.0
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
                self.controller.stop()
                self.print_stuff()

            elif inp == 'd':
                self.d += 10
                self.a = 0.0
                self.controller.right(self.d)
                self.print_stuff()

            elif inp == 'x':
                self.x += 10.0
                self.w = 0.0
                self.controller.back(self.x)
                self.print_stuff()

            elif inp == 'q':
                self.q += 10.0
                self.e = 0
                self.controller.turn_left(self.q)
                self.print_stuff()

            elif inp == 'e':
                self.e += 10.0
                self.q = 0
                self.controller.turn_right(self.e)
                self.print_stuff()

            elif inp == 't':
                self.controller.turn_right(100)
                time.sleep(20.7)
                self.controller.stop()
            else:
                msg = Twist()
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0

                msg.linear.x = 0.0
                self.publisher_.publish(msg)
                self.print_stuff()
        


def main(args=None):
    rclpy.init(args=args)

    keyboard_publisher = KeyboardPublisher()

    rclpy.spin(keyboard_publisher)

    keyboard_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()