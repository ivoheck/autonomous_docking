#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#subscriber KLasse https://docs.ros.org/en/kinetic/api/rospy/html/rospy.topics.Subscriber-class.html


import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile


class Dock(Node):

    def __init__(self):
        super().__init__('dock')
        self.distance = []
        self.tollerace = 0.001
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, '/motor_controller', 10)
        
    def listener_callback(self, msg):
        #TODO: docking abrechen wenn sich die front distans nicht mehr ändert
        front = msg.ranges[0]
        if self.check_movement(front):
            self.stop()
            self.get_logger().info('Docking Failed')
            rclpy.shutdown()
            return
        
        if front >= 0.185:
            self.front(speed=10.0)
        elif front >= 0.167:
            self.front(speed=30.0)
        else:
            self.stop()
            self.get_logger().info('Docking Succeed')
            rclpy.shutdown()
            return

    def check_movement(self,front):
        self.distance.append(front)
        
        if len(self.distance) > 30:
            self.distance.pop(0)
            if abs(self.distance[0] - self.distance[29]) < self.tollerace:
                return True
            else:
                return False

        return False

    def front(self,speed):
        msg = Twist()
        msg.linear.x = speed
        self.publisher_.publish(msg)

    def stop(self):
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