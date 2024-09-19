#https://pypi.org/project/RPi.GPIO/
#https://sourceforge.net/p/raspberry-gpio-python/wiki/BasicUsage/

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import RPi.GPIO as GPIO


class LockController(Node):

    def __init__(self):
        super().__init__('lock_controller')
        self.subscription = self.create_subscription(
            Bool,
            '/lock',
            self.controll_lock,
            10)
        self.subscription

        self.GPIO = 26
        self.GPIO_setup()
        GPIO.output(self.GPIO, GPIO.HIGH)
        print('HIGH')

    def GPIO_setup(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.GPIO, GPIO.OUT, initial=GPIO.LOW)

    def controll_lock(self,msg):
        if msg.data:
            GPIO.output(self.GPIO, GPIO.HIGH)
        else:
            GPIO.output(self.GPIO, GPIO.LOW)

        
def main(args=None):
    rclpy.init(args=args)
    lock_controller = LockController()
    rclpy.spin(lock_controller)
    lock_controller.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()