import sys
import json

from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import time

class Dock(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        #TODO:dynamicly
        self.probe_l = 507
        self.probe_r = 517
        
        self.probe_tollerace = 50
        self.publisher_ = self.create_publisher(Twist, '/motor_controller', 10)

        self.cli = self.create_client(Trigger, '/docking_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.send_request)

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        response = future.result()
        message = response.message
        message = json.loads(message)
        lidar_front = message[0]
        probe_left = message[1][0] - self.probe_l
        probe_right = message[1][1] -self.probe_r

        #print(lidar_front)
        #print(probe_left)
        #print(probe_right)

        if (lidar_front >= 0.185 or lidar_front == 'inf') and probe_left == 0 and probe_right == 0:
            self.front(speed=5.0)

        elif lidar_front >= 0.167 and probe_left == 0 and probe_right == 0:
            self.front(speed=30.0)

        elif lidar_front < 0.167:
            self.stop()
            rclpy.shutdown()
            return

        elif probe_left != 0 and probe_right != 0:
            self.handle_both_probe(probe_left,probe_right)

        elif probe_left != 0:
            self.handle_left_probe(probe_left)

        elif probe_right != 0:
            self.handle_right_probe(probe_right)

    def handle_both_probe(self,probe_left,probe_right):
        #Beide fühler zeigen nach rechts (von hinten geschaut)
        if probe_left < 0 and probe_right < 0:
            if abs(abs(probe_left) - abs(probe_right)) < self.probe_tollerace:
                self.rigth(speed=5.0)
            
        #Beide fühler zeigen nach links (von hinten geschaut)
        elif probe_left > 0 and probe_right > 0:
            if abs(abs(probe_left) - abs(probe_right)) < self.probe_tollerace:
                self.left(speed=5.0)

        else:
            print('problem')

    def handle_left_probe(self,probe_left):
        if probe_left < 0:
            if probe_left > -100:
                self.turn_right(speed=5.0)
            else:
                self.rigth(speed=5.0)
        else:
            if probe_left < 100:
                self.turn_left(speed=5.0)
            else:
                self.left(speed=5.0)
            

    def handle_right_probe(self,probe_rigth):
        print('handle right probe')
        if probe_rigth < 0:
            if probe_rigth > -100:
                self.turn_left(speed=5.0)
            else:
                self.rigth(speed=5.0)
        else:
            if probe_rigth < 100:
                self.turn_right(speed=5.0)
            else:
                self.left(speed=5.0)
            

    def front(self,speed):
        print('front')
        msg = Twist()
        msg.linear.x = speed
        self.publisher_.publish(msg)

    def left(self,speed):
        print('left')
        msg = Twist()
        msg.linear.y = speed
        self.publisher_.publish(msg)

    def rigth(self,speed):
        msg = Twist()
        msg.linear.y = -(speed)
        self.publisher_.publish(msg)
        print('right 2')

    def turn_left(self,speed):
        print('turn left')
        msg = Twist()
        msg.angular.z = speed
        self.publisher_.publish(msg)

    def turn_right(self,speed):
        print('turn rigth')
        msg = Twist()
        msg.angular.z = -(speed)
        self.publisher_.publish(msg)

    def stop(self):
        print('stop')
        msg = Twist()
        msg.angular.z = 0.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    dock = Dock()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(dock)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            dock.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()