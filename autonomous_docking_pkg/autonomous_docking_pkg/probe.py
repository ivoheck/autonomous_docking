#https://pyserial.readthedocs.io/en/latest/shortintro.html
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool


class Probe(Node):

    def __init__(self):
        super().__init__('probe')
        self.publisher_ = self.create_publisher(Int32MultiArray, '/probe', 10)
        self.serial = serial.Serial('/dev/usb_serial_arduino_uno', 9600)#udev regel für arduino uno

        self.subscription = self.create_subscription(
            Bool,
            '/lock',
            self.controll_lock,
            10)
        self.subscription

        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.check_probe)

    def controll_lock(self,msg):
        if msg.data:
            #open lock
            self.serial.write('1'.encode())
        else:
            #close lock
            self.serial.write('0'.encode())

    def check_probe(self):
        try:
            line = self.serial.readline().decode('utf-8')
            values = line.split('-')
            valuex_l,valuey_l,valuex_m,valuey_m,valuex_r,valuey_r = int(values[0]),int(values[1]),int(values[2]),int(values[3]),int(values[4]),int(values[5])
            
            msg = Int32MultiArray()
            msg.data = [valuex_l,valuey_l,valuex_m,valuey_m,valuex_r,valuey_r]
            print([valuex_l,valuey_l,valuex_m,valuey_m,valuex_r,valuey_r])
            self.publisher_.publish(msg)
            
        except Exception as e:
            print(e)

        return 
        
def main(args=None):
    rclpy.init(args=args)
    probe = Probe()
    rclpy.spin(probe)
    probe.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()