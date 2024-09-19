

#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#subscriber KLasse https://docs.ros.org/en/kinetic/api/rospy/html/rospy.topics.Subscriber-class.html


import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import time


class DrivePath(Node):

    def __init__(self):
        super().__init__('drive_path')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.commands = [['x', -10.0, 1.2997207641601562], ['x', -10.0, 1.641740083694458], ['x', -10.0, 1.9525279998779297], ['x', -10.0, 2.2481415271759033], ['x', -10.0, 2.5521931648254395], ['x', -10.0, 2.9085419178009033], ['x', -10.0, 3.7940986156463623], ['x', -10.0, 4.244967460632324], ['x', -10.0, 4.660339832305908], ['x', -10.0, 5.186996698379517], ['s', 0.0, 17.106407165527344], ['z', -10.0, 18.04305338859558], ['z', -10.0, 18.37646508216858], ['z', -10.0, 18.67277479171753], ['z', -10.0, 19.006441831588745], ['z', -10.0, 19.28032922744751], ['z', -10.0, 19.572543382644653], ['z', -10.0, 19.895246267318726], ['z', -10.0, 20.91174006462097], ['z', -10.0, 21.60683035850525], ['s', 0.0, 23.79505157470703], ['x', 10.0, 24.271976709365845], ['x', 10.0, 24.586270093917847], ['x', 10.0, 24.86092758178711], ['x', 10.0, 25.147150993347168], ['x', 10.0, 25.461992263793945], ['x', 10.0, 25.786250114440918], ['x', 10.0, 26.06305742263794], ['x', 10.0, 26.426721572875977], ['x', 10.0, 26.816730976104736], ['x', 10.0, 27.330610275268555], ['z', -10.0, 35.18777370452881], ['z', -10.0, 35.58814740180969], ['s', 0.0, 37.386016607284546], ['x', 10.0, 37.72355937957764], ['x', 10.0, 37.96491360664368], ['x', 10.0, 38.176756143569946], ['x', 10.0, 39.05240750312805], ['x', 10.0, 39.443657636642456], ['x', 10.0, 39.724255323410034], ['x', 10.0, 39.97089219093323], ['x', 10.0, 40.234182596206665], ['x', 10.0, 40.467262268066406], ['x', 10.0, 40.69482350349426], ['z', 10.0, 50.31126022338867], ['z', 10.0, 50.58314871788025], ['z', 10.0, 50.86945557594299], ['z', 10.0, 51.07339358329773], ['z', 10.0, 51.33461904525757], ['z', 10.0, 51.55006670951843], ['z', 10.0, 52.508821964263916], ['z', 10.0, 52.77491092681885], ['z', 10.0, 52.96358132362366], ['z', 10.0, 53.15902352333069], ['s', 0.0, 55.74911069869995]]

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.drive_path()

    def drive_path(self):
        start_time = time.time()
        for command in self.commands:
            msg = Twist()
            while time.time() - start_time < command[2]:
                pass
            print(command)
            if command[0] == 'x':
                self.x += command[1]
                msg.linear.x = self.x
                self.publisher_.publish(msg)
            elif command[0] == 'y':
                self.y += command[1]
                msg.linear.y = self.y
                self.publisher_.publish(msg)
            elif command[0] == 'z':
                self.z += command[1]
                msg.angular.z = self.z
                self.publisher_.publish(msg)

            elif command[0] == 's':
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
                msg.linear.x = 0.0
                self.publisher_.publish(msg)
                
        rclpy.shutdown()

        

def main(args=None):
    rclpy.init(args=args)

    drive_path = DrivePath()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(drive_path)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            drive_path.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()