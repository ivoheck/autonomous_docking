import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import time


class KeyboardPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.start_time = None
        self.commands = []
        self.start_loop()

    def print_stuff(self):
        print('x',self.x)
        print('y',self.y)
        print('z',self.z)

    def start_loop(self):
        while True:
            inp = str(input('key: '))

            if inp == 'w':
                msg = Twist()
                self.x += 10.0
                msg.linear.x = self.x
                self.publisher_.publish(msg)
                self.commands.append(['x', 10.0, time.time() - self.start_time])
                self.print_stuff()

            elif inp == 'a':
                msg = Twist()
                self.y += 10.0
                msg.linear.y = self.y
                self.publisher_.publish(msg)
                self.commands.append(['y', 10.0, time.time() - self.start_time])
                self.print_stuff()

            elif inp == 's':
                msg = Twist()
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0

                msg.linear.x = 0.0
                self.publisher_.publish(msg)
                self.commands.append(['s', 0.0, time.time() - self.start_time])
                self.print_stuff()

            elif inp == 'd':
                msg = Twist()
                self.y -= 10 
                msg.linear.y = self.y
                self.publisher_.publish(msg)
                self.commands.append(['y', -10.0, time.time() - self.start_time])
                self.print_stuff()

            elif inp == 'x':
                msg = Twist()
                self.x -= 10.0
                msg.linear.x = self.x
                self.publisher_.publish(msg)
                self.commands.append(['x', -10.0, time.time() - self.start_time])
                self.print_stuff()

            elif inp == 'q':
                msg = Twist()
                self.z += 10.0
                msg.angular.z = self.z
                self.publisher_.publish(msg)
                self.commands.append(['z', 10.0, time.time() - self.start_time])
                self.print_stuff()

            elif inp == 'e':
                msg = Twist()
                self.z -= 10.0
                msg.angular.z = self.z
                self.publisher_.publish(msg)
                self.commands.append(['z', -10.0, time.time() - self.start_time])
                self.print_stuff()

            elif inp == 'r':
                msg = Twist()
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0

                msg.linear.x = 0.0
                self.publisher_.publish(msg)
                self.print_stuff()
                self.start_time = time.time()
                self.commands = []
                self.timestamps = []
                print('start recording')

            elif inp == 't':
                msg = Twist()
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0

                msg.linear.x = 0.0
                self.publisher_.publish(msg)
                self.print_stuff()
                print('stop recording')
                print(self.commands)

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


commands = [['x', -10.0, 1.2997207641601562], ['x', -10.0, 1.641740083694458], ['x', -10.0, 1.9525279998779297], ['x', -10.0, 2.2481415271759033], ['x', -10.0, 2.5521931648254395], ['x', -10.0, 2.9085419178009033], ['x', -10.0, 3.7940986156463623], ['x', -10.0, 4.244967460632324], ['x', -10.0, 4.660339832305908], ['x', -10.0, 5.186996698379517], ['x', 0.0, 17.106407165527344], ['z', -10.0, 18.04305338859558], ['z', -10.0, 18.37646508216858], ['z', -10.0, 18.67277479171753], ['z', -10.0, 19.006441831588745], ['z', -10.0, 19.28032922744751], ['z', -10.0, 19.572543382644653], ['z', -10.0, 19.895246267318726], ['z', -10.0, 20.91174006462097], ['z', -10.0, 21.60683035850525], ['x', 0.0, 23.79505157470703], ['x', 10.0, 24.271976709365845], ['x', 10.0, 24.586270093917847], ['x', 10.0, 24.86092758178711], ['x', 10.0, 25.147150993347168], ['x', 10.0, 25.461992263793945], ['x', 10.0, 25.786250114440918], ['x', 10.0, 26.06305742263794], ['x', 10.0, 26.426721572875977], ['x', 10.0, 26.816730976104736], ['x', 10.0, 27.330610275268555], ['z', -10.0, 35.18777370452881], ['z', -10.0, 35.58814740180969], ['x', 0.0, 37.386016607284546], ['x', 10.0, 37.72355937957764], ['x', 10.0, 37.96491360664368], ['x', 10.0, 38.176756143569946], ['x', 10.0, 39.05240750312805], ['x', 10.0, 39.443657636642456], ['x', 10.0, 39.724255323410034], ['x', 10.0, 39.97089219093323], ['x', 10.0, 40.234182596206665], ['x', 10.0, 40.467262268066406], ['x', 10.0, 40.69482350349426], ['z', 10.0, 50.31126022338867], ['z', 10.0, 50.58314871788025], ['z', 10.0, 50.86945557594299], ['z', 10.0, 51.07339358329773], ['z', 10.0, 51.33461904525757], ['z', 10.0, 51.55006670951843], ['z', 10.0, 52.508821964263916], ['z', 10.0, 52.77491092681885], ['z', 10.0, 52.96358132362366], ['z', 10.0, 53.15902352333069], ['x', 0.0, 55.74911069869995]]