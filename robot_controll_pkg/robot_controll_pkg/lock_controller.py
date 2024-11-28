#https://pyserial.readthedocs.io/en/latest/shortintro.html
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class LockController(Node):

    def __init__(self):
        super().__init__('lock_controller')
        self.serial = serial.Serial('/dev/usb_serial_arduino_uno', 9600)#udev regel für arduino uno

        self.subscription = self.create_subscription(
            Bool,
            '/lock',
            self.controll_lock,
            1)
        self.subscription

    def controll_lock(self,msg):
        if msg.data:
            #lock öffnen
            self.serial.write('1'.encode())
        else:
            #lock schließen
            self.serial.write('0'.encode())

        
def main(args=None):
    rclpy.init(args=args)
    lock_controller = LockController()
    rclpy.spin(lock_controller)
    lock_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()