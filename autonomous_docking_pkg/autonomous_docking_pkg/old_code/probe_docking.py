import sys
import json

from autonomous_docking_pkg import motor_controller
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
from custom_interfaces.msg import DockTrigger
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Dock(Node):
    def __init__(self):
        super().__init__('dock')
        
        self.probe_tollerace = 20
        self.publisher_lock = self.create_publisher(Bool, '/lock', 1)
        self.publisher_current_state = self.create_publisher(String, '/current_state', 1)
        self.publisher_start_docking = self.create_publisher(DockTrigger, '/start_dock', 1)
        self.controller = motor_controller.MotorControllerHelper()

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.lidar_subscription
        self.lidar_ranges = None
        self.lidar_list = []
        self.check_value_len = 15
        self.check_value_dis = 0.001

        self.timer = self.create_timer(0.1, self.docking)

    def open_lock(self):
        msg = Bool()
        msg.data = True
        self.publisher_lock.publish(msg)

    def close_lock(self):
        msg = Bool()
        msg.data = False
        self.publisher_lock.publish(msg)

    def lidar_callback(self,msg):
        self.lidar_ranges = msg.ranges

    def docking(self):
        lidar_front = self.lidar_ranges[0]

        self.lidar_list.append(lidar_front)

        if len(self.lidar_list) > self.check_value_len:
            self.lidar_list.pop(0)

            print(abs(self.lidar_list[0] - self.lidar_list[self.check_value_len -1]) )
            if abs(self.lidar_list[0] - self.lidar_list[self.check_value_len -1]) < self.check_value_dis:
                self.close_lock()
                self.controller.stop()

                self.get_logger().info('Docking failed')
                
                msg = String()
                msg.data = 'docking_fail'
                self.publisher_current_state.publish(msg)

                msg = DockTrigger()
                msg.trigger = False
                self.publisher_start_docking.publish(msg)

                rclpy.shutdown()
                return

        if (lidar_front >= 0.185 or lidar_front == 'inf'): #and offset == 0:
            self.controller.front(percent=5.0)

        elif lidar_front >= 0.175: # and offset == 0:
            self.open_lock()
            self.controller.front(percent=10.0)

        elif lidar_front >= 0.167: # and offset == 0:
            self.controller.front(percent=100.0)

        #Docking erfolgreich
        elif lidar_front < 0.167:
            self.close_lock()
            self.controller.stop()
                
            msg = String()
            msg.data = 'docked'
            self.publisher_current_state.publish(msg)

            msg = DockTrigger()
            msg.trigger = False
            self.publisher_start_docking.publish(msg)

            self.get_logger().info('Docking success')

            rclpy.shutdown()
            return


    #LLM
    def handle_response(self, future):
        try:
            response = future.result()
            message = response.message
            message = json.loads(message)
            #LLM
            lidar_front = message[0]
            probey_left = message[1][1] - self.probey_l
            probey_right = message[1][5] -self.probey_r
            
            probex_mid = message[1][2] - self.probex_m
            
        #TODO: evtl speed dynamisch machen
            #if probex_mid > 1:
            #    self.controller.left(percent=5.0)
            #    print('3333lesfdfjlds')
            #    return
            #elif probex_mid < -1:
            #    self.controller.right(percent=5.0)
            #    print('lesfdfjlds')
            #    return
            

        #print(lidar_front)
        #print(probey_left)
        #print(probey_right)

            #offset = self.get_offset(probey_left,probey_right)
            #print(offset)
            #self.get_logger().info(str(offset))

            if (lidar_front >= 0.185 or lidar_front == 'inf'): #and offset == 0:
                self.controller.front(percent=5.0)

            elif lidar_front >= 0.175: # and offset == 0:
                self.open_lock()
                self.controller.front(percent=10.0)

            elif lidar_front >= 0.167: # and offset == 0:
                self.controller.front(percent=100.0)

            #Docking erfolgreich
            elif lidar_front < 0.167:
                self.close_lock()
                self.controller.stop()
                
                msg = String()
                msg.data = 'docked'
                self.publisher_current_state.publish(msg)

                msg = DockTrigger()
                msg.trigger = False
                self.publisher_start_docking.publish(msg)

                rclpy.shutdown()
                return

            #Roboter nach rechts gedreht
            #elif offset < 0:
            #    self.controller.drive_curve(direction=-1,percent_x=4.0,percent_z=2.0)

            #Roboter nach links gedreht
            #elif offset > 0:
            #    self.controller.drive_curve(direction=1,percent_x=4.0,percent_z=2.0)

        except:
            pass
        
    def get_offset(self,probey_left,probey_right):
        #negatver offset -> rechts ist größer (probe stärker nach oben geneigt)
        offset = abs(probey_left) - abs(probey_right)
        #Kleinere abweichungen von 0 werden ignoriert
        if abs(offset) <= 3:
            offset = 0
        return offset


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