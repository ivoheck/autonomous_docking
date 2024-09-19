#https://tutorials-raspberrypi.de/arduino-raspberry-pi-miteinander-kommunizieren-lassen/
#https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
#https://wiki.ros.org/Robots/TIAGo/Tutorials/motions/cmd_vel

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

        self.serial = serial.Serial('/dev/usb_serial_arduino_mega', 9600) #udev regel für arduino mega

    def x_linear_movement(self, linear_x):
        negativ = True if linear_x < 0 else False

        if negativ:
            sign = '-'
        else:
            sign = '+'

        abs_value = abs(int(linear_x))

        if abs_value >= 100:
            abs_value = 100

        if len(str(abs_value)) == 3:
            zero = ''
        elif len(str(abs_value)) == 2:
            zero = '0'
        elif len(str(abs_value)) == 1:
            zero = '00'

        command = f'{sign}{zero+str(abs_value)}{sign}{zero+str(abs_value)}{sign}{zero+str(abs_value)}{sign}{zero+str(abs_value)}'
        self.serial.write(command.encode())

    def y_linear_movement(self,linear_y):
        negativ = True if linear_y < 0 else False

        abs_value = abs(int(linear_y))

        if abs_value >= 100:
            abs_value = 100

        plus = '+'
        minus = '-'

        if len(str(abs_value)) == 3:
            zero = ''
        elif len(str(abs_value)) == 2:
            zero = '0'
        elif len(str(abs_value)) == 1:
            zero = '00'

        #Bewegung nach links
        if negativ:
            command = f'{minus}{zero+str(abs_value)}{plus}{zero+str(abs_value)}{minus}{zero+str(abs_value)}{plus}{zero+str(abs_value)}'
            self.serial.write(command.encode())

        #Bewegung nach rechts
        else:
            command = f'{plus}{zero+str(abs_value)}{minus}{zero+str(abs_value)}{plus}{zero+str(abs_value)}{minus}{zero+str(abs_value)}'
            self.serial.write(command.encode())


    def z_angular_movement(self,angular_z):
        negativ = True if angular_z < 0 else False

        abs_value = abs(int(angular_z))

        if abs_value >= 100:
            abs_value = 100

        plus = '+'
        minus = '-'

        if len(str(abs_value)) == 3:
            zero = ''
        elif len(str(abs_value)) == 2:
            zero = '0'
        elif len(str(abs_value)) == 1:
            zero = '00'

        #Mit dem uhrzeigersinn 
        if negativ:
            command = f'{plus}{zero+str(abs_value)}{plus}{zero+str(abs_value)}{minus}{zero+str(abs_value)}{minus}{zero+str(abs_value)}'
            self.serial.write(command.encode())
            
        #Gegen den Uhrzeigersinn
        else:
            command = f'{minus}{zero+str(abs_value)}{minus}{zero+str(abs_value)}{plus}{zero+str(abs_value)}{plus}{zero+str(abs_value)}'
            self.serial.write(command.encode())

    def x_linear_and_z_angular_movement(self,linear_x,angular_z):
        negativ_x = True if linear_x < 0 else False

        if negativ_x:
            sign = '-'
        else:
            sign = '+'

        abs_value = abs(int(linear_x))

        if abs_value >= 100:
            abs_value = 100

        if len(str(abs_value)) == 3:
            zero = ''
        elif len(str(abs_value)) == 2:
            zero = '0'
        elif len(str(abs_value)) == 1:
            zero = '00'

        left_side = abs_value
        rigth_side = abs_value

        negativ_z = True if angular_z < 0 else False

        #Rechts drehung
        if negativ_z:
            rigth_side -= int(abs(angular_z)*0.2)
            left_side += int(abs(angular_z)*0.2)

        #Links drehung
        else:
            rigth_side += int(abs(angular_z)*0.2)
            left_side -= int(abs(angular_z)*0.2)

        command = f'{sign}{zero+str(left_side)}{sign}{zero+str(left_side)}{sign}{zero+str(rigth_side)}{sign}{zero+str(rigth_side)}'
        self.serial.write(command.encode())

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
        self.get_logger().info(str(command))
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
        WHEEL_GEOMETRY = 29.1 #WHEEL_GEOMETRY = (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) / 2

        front_left = (x - y - rot * WHEEL_GEOMETRY)# / WHEEL_RADIUS
        front_right = (x + y + rot * WHEEL_GEOMETRY)# / WHEEL_RADIUS
        back_left = (x + y - rot * WHEEL_GEOMETRY)# / WHEEL_RADIUS
        back_right = (x - y + rot * WHEEL_GEOMETRY)# / WHEEL_RADIUS
        #https://ecam-eurobot.github.io/Tutorials/software/mecanum/mecanum.html

        self.get_logger().info(str(('front_left',front_left)))
        self.get_logger().info(str(('front_left',front_right)))
        self.get_logger().info(str(('front_left',back_left)))
        self.get_logger().info(str(('front_left',back_right)))

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

        self.get_logger().info(str(('front_left',int(p_front_left))))
        self.get_logger().info(str(('front_left',int(p_front_right))))
        self.get_logger().info(str(('front_left',int(p_back_left))))
        self.get_logger().info(str(('front_left',int(p_back_right))))

        return int(p_front_left),int(p_front_right),int(p_back_left),int(p_back_right)

    def listener_callback_twist(self, msg):
        front_left,front_right,back_left,back_right = self.convert(msg)
        p_front_left,p_front_right,p_back_left,p_back_right = self.callc_percent(front_left,front_right,back_left,back_right)
        self.percent_movement(p_front_left,p_front_right,p_back_left,p_back_right)
        self.get_logger().info(str('test'))
        '''
        #umrechung von m/s und rad/s in prozent

        #x vorne-hinten
        linear_x = (msg.linear.x / 0.085)*100
        #y links-rechts
        linear_y = (msg.linear.y / 0.085)*100
        #z drehen links-rechst
        angular_z = (msg.angular.z / 0.3035355212505)*100

        #Serielle nachrichten sind wie follgt aufgebaut(von oben/hinten): rad-links-unten rad-links-oben rad-recht-oben rad-rechts-unten
        #für jedes rad ein vorzeichen(plus oder minus) und 3 zeiche für die geschwindikeit(von 000 bist 100 also 0%-100% Geschwindikeit) 
        
        #Reine X-achsen bewegung
        if linear_x != 0.0 and linear_y == 0.0 and angular_z == 0.0:
            self.x_linear_movement(linear_x=linear_x)

        #Reine y-achsen bewegung
        elif linear_x == 0.0 and linear_y != 0.0 and angular_z == 0.0:
            self.y_linear_movement(linear_y=linear_y)

        #Reine Dreh bewegung
        elif linear_x == 0.0 and linear_y == 0.0 and angular_z != 0.0:
            self.z_angular_movement(angular_z=angular_z)

        #Linear X-achse und drehung
        elif linear_x != 0.0 and linear_y == 0.0 and angular_z != 0.0:
            self.x_linear_and_z_angular_movement(linear_x=linear_x,angular_z=angular_z)

        #Lineare Y-achse und drehung
        elif linear_x == 0.0 and linear_y != 0.0 and angular_z != 0.0:
            self.y_linear_and_z_angular_movement(linear_y=linear_y,angular_z=angular_z)

        else:
            self.stop_all()

        '''


#TODO: für mehr prozentstufen die zahlen berechnen für mehr genauigkeit
class MotorControllerHelper(Node):
    def __init__(self):
        super().__init__('motor_controller_helper')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

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
