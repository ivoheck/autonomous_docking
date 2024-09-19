import sys
import json

from autonomous_docking_pkg import motor_controller
from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class Dock(Node):
    def __init__(self):
        super().__init__('dock')
        #TODO:dynamicly
        self.probey_l = 508
        self.probey_r = 509
        self.probex_m = 515
        
        self.probe_tollerace = 20
        self.publisher_lock = self.create_publisher(Bool, '/lock', 10)
        self.controller = motor_controller.MotorControllerHelper()

        self.cli = self.create_client(Trigger, '/docking_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.send_request)

    def open_lock(self):
        msg = Bool()
        msg.data = True
        self.publisher_lock.publish(msg)

    def close_lock(self):
        msg = Bool()
        msg.data = False
        self.publisher_lock.publish(msg)

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            message = response.message
            message = json.loads(message)
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

            offset = self.get_offset(probey_left,probey_right)
            print(offset)
            self.get_logger().info(str(offset))

            if (lidar_front >= 0.185 or lidar_front == 'inf') and offset == 0:
                self.controller.front(percent=5.0)

            elif lidar_front >= 0.175 and offset == 0:
                self.open_lock()
                self.controller.front(percent=10.0)

            elif lidar_front >= 0.167 and offset == 0:
                self.controller.front(percent=100.0)

            #Docking erfolgreich
            elif lidar_front < 0.167:
                self.close_lock()
                self.controller.stop()
                rclpy.shutdown()
                return

            #Roboter nach rechts gedreht
            elif offset < 0:
                self.controller.drive_curve(direction=-1,percent_x=4.0,percent_z=2.0)

            #Roboter nach links gedreht
            elif offset > 0:
                self.controller.drive_curve(direction=1,percent_x=4.0,percent_z=2.0)

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