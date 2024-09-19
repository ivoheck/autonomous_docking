#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

#subscriber KLasse https://docs.ros.org/en/kinetic/api/rospy/html/rospy.topics.Subscriber-class.html


from autonomous_docking_pkg import motor_controller

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

import math


#27,8 - 29,3

class FaceWall(Node):

    def __init__(self):
        super().__init__('face_wall')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tolerance = 0.001
        #self.front_limit = 0.1
        self.state = True
        #Diese variable wird zum kallibriren den Lidar sensors verwendet
        self.lidar_offset = 0.0008181821216236461
        self.check = 0
        self.checks = 3
        self.controller = motor_controller.MotorControllerHelper()
        
    def listener_callback(self, msg):
        #TODO: evtl mehr datenpunkte nehmen und toleranz abhängig von der front range machen
        front = msg.ranges[0]
        l_1, l_2, l_6, l_7, l_11 = msg.ranges[1], msg.ranges[2], msg.ranges[6], msg.ranges[7], msg.ranges[11]
        r_1, r_2, r_6, r_7, r_11 = msg.ranges[719], msg.ranges[718], msg.ranges[714], msg.ranges[715], msg.ranges[709]

        l_value = l_1 + l_2 + l_6 + l_7 + l_11
        r_value = r_1 + r_2 + r_6 + r_7 + r_11

        print(front)

        diff = l_value -r_value
        if math.isinf(abs(diff)):
            return
        
        diff -= self.lidar_offset

        if abs(diff) < self.tolerance:
            self.controller.stop()
            if self.check >= self.checks:
                rclpy.shutdown()
                return
            else:
                self.check += 1
                return 
            #TODO: nach dem stoppen nochmal waren und nochmal überprüfen
            #TODO: an die mauer ran fahren evtl mit nochmal koriktur bewegungen
            #self.state = False
            #if front <= self.front_limit:
            #    self.stop()
            #else:
            #    self.front(speed=0.05)
            #self.timer = self.create_timer(1, self.timer_callback)

        else:
            if diff > 0:
                self.controller.turn_right(percent=5.0)
                #time.sleep(0.1)
            else:
                self.controller.turn_left(percent=5.0)
                #time.sleep(0.1)
                

        #TODO: dynamisch den turn speed je nach abweichung machen
        #TODO: bei dem richtigen roboter währe es gut ein fedback loop zu haben der bescheid sagt wenn eine bestimmte bewegungs operation abgeschlossen ist
        #TODO: man müsste quasi einen weiteren subscriber für die bewegungs steuerung haben

    def turn_left(self,speed):
        print('left')
        msg = Twist()
        msg.angular.z = speed
        self.publisher_.publish(msg)

    def turn_right(self,speed):
        print('rigth')
        msg = Twist()
        msg.angular.z = -(speed)
        self.publisher_.publish(msg)

    def front(self,speed):
        print('front')
        msg = Twist()
        msg.linear.x = speed
        self.publisher_.publish(msg)

    def stop(self):
        print('stop')
        msg = Twist()
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    face_wall = FaceWall()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(face_wall)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            face_wall.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()