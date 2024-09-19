#https://tutorials-raspberrypi.de/arduino-raspberry-pi-miteinander-kommunizieren-lassen/
#https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
#https://wiki.ros.org/Robots/TIAGo/Tutorials/motions/cmd_vel
#https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html

import serial
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist

# +x -> nach vorne
# -x -> nach hinten

# +y -> nach links
# -y -> nach rechts

# +z -> gegen uhrzeigersinn
# -z -> im uhrzeigersinn

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription_twist = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback_twist,
            10)
        self.subscription_twist  # prevent unused variable warning

        self.publisher_enconder = self.create_publisher(Twist, '/encoder', 1)

        self.serial = serial.Serial('/dev/usb_serial_arduino_mega', 9600) #udev regel für arduino mega

    def callc_encoder(self,p_front_left,p_front_right,p_back_left,p_back_right):
        front_left = (p_front_left/100) * 0.085
        front_right = (p_front_right/100) * 0.085
        back_left = (p_back_left/100) * 0.085
        back_right = (p_back_right/100) * 0.085

        WHEEL_GEOMETRY = 0.291
        
        x = (front_left + front_right + back_left + back_right) / 4
        y = (-front_left + front_right + back_left - back_right) / 4
        z = (-front_left + front_right - back_left + back_right) / (4*WHEEL_GEOMETRY)

        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = z
        self.publisher_enconder.publish(msg)

    def get_sign(self,percent):
        if percent >= 0:
            return '+'
        else:
            return '-'

    def get_string(self,percent):
        if len(str(abs(percent))) == 3:
            return  '' + str(abs(percent))
        elif len(str(abs(percent))) == 2:
            return '0' + str(abs(percent))
        elif len(str(abs(percent))) == 1:
            return '00' + str(abs(percent))
        
        return '000'

    def percent_movement(self,p_front_left,p_front_right,p_back_left,p_back_right):
        sign_f_l = self.get_sign(p_front_left)
        sign_f_r = self.get_sign(p_front_right)
        sign_b_l = self.get_sign(p_back_left)
        sign_b_r = self.get_sign(p_back_right)

        string_f_l = self.get_string(p_front_left)
        string_f_r = self.get_string(p_front_right)
        string_b_l = self.get_string(p_back_left)
        string_b_r = self.get_string(p_back_right)
        
        command = f'{sign_b_l}{string_b_l}{sign_f_l}{string_f_l }{sign_f_r }{string_f_r}{sign_b_r}{string_b_r}'
        self.serial.write(command.encode())

    def stop_all(self):
        command = '+000+000+000+000'
        self.serial.write(command.encode())

    #https://ecam-eurobot.github.io/Tutorials/software/mecanum/mecanum.html
    #konvertiert twist massage kompnenten in die einzelnen beschleunigungen für die einzelen räder
    def convert(self,move):
        x = move.linear.x
        y = move.linear.y
        rot = move.angular.z

        WHEEL_RADIUS = 0.037
        WHEEL_GEOMETRY = 0.291 #WHEEL_GEOMETRY = (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) / 2

        front_left = (x - y - rot * WHEEL_GEOMETRY)# / WHEEL_RADIUS
        front_right = (x + y + rot * WHEEL_GEOMETRY)# / WHEEL_RADIUS
        back_left = (x + y - rot * WHEEL_GEOMETRY)# / WHEEL_RADIUS
        back_right = (x - y + rot * WHEEL_GEOMETRY)# / WHEEL_RADIUS
        #https://ecam-eurobot.github.io/Tutorials/software/mecanum/mecanum.html

        #Einheit: m/s
        return front_left ,front_right ,back_left ,back_right 

    def callc_percent(self,front_left,front_right,back_left,back_right):
        p_front_left = int((front_left/0.085)*100)
        p_front_right = int((front_right/0.085)*100)
        p_back_left = int((back_left/0.085)*100)
        p_back_right = int((back_right/0.085)*100)

        max_v = max(abs(p_front_left),abs(p_front_right),abs(p_back_left),abs(p_back_right))
        if max_v > 100:
           p_front_left  = (p_front_left / max_v) * 100
           p_front_right = (p_front_right / max_v) * 100
           p_back_left = (p_back_left / max_v) * 100
           p_back_right = (p_back_right / max_v) * 100

        return int(p_front_left),int(p_front_right),int(p_back_left),int(p_back_right)

    def listener_callback_twist(self, msg):
        front_left,front_right,back_left,back_right = self.convert(msg)
        p_front_left,p_front_right,p_back_left,p_back_right = self.callc_percent(front_left,front_right,back_left,back_right)
        self.percent_movement(p_front_left,p_front_right,p_back_left,p_back_right)

        self.callc_encoder(p_front_left,p_front_right,p_back_left,p_back_right)

#TODO: für mehr prozentstufen die zahlen berechnen für mehr genauigkeit
class MotorControllerHelper(Node):
    def __init__(self):
        super().__init__('motor_controller_helper')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)

    def callculate_linear_movement():
        pass
        
    def front(self,percent):
        msg = Twist()
        msg.linear.x = 0.085*(percent/100)
        self.publisher_.publish(msg)

    def back(self,percent):
        msg = Twist()
        msg.linear.x = -(0.085*(percent/100))
        self.publisher_.publish(msg)

    def left(self,percent):
        msg = Twist()
        msg.linear.y = 0.085*(percent/100)
        self.publisher_.publish(msg)

    def right(self,percent):
        msg = Twist()
        msg.linear.y = -(0.085*(percent/100))
        self.publisher_.publish(msg)

    def turn_right(self,percent):
        msg = Twist()
        msg.angular.z = -(0.3035355212505*(percent/100))
        self.publisher_.publish(msg)

    def turn_left(self,percent):
        msg = Twist()
        msg.angular.z = 0.3035355212505*(percent/100)
        self.publisher_.publish(msg)

    def drive_curve(self,direction,percent_x,percent_z):
        msg = Twist()
        msg.angular.z = 0.3035355212505*((percent_z * direction)/100)
        msg.linear.x = 0.085*(percent_x/100)
        self.publisher_.publish(msg)

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
