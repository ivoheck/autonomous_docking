#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://www.geeksforgeeks.org/how-to-capture-a-image-from-webcam-in-python/
#https://www.geeksforgeeks.org/webcam-qr-code-scanner-using-opencv/
#https://www.geeksforgeeks.org/reading-images-in-python/
#https://note.nkmk.me/en/python-opencv-qrcode/

import rclpy
from rclpy.node import Node
from custom_interfaces.msg import QrPos
from custom_interfaces.msg import DockTrigger
import cv2
import time

#QR-Code-Benennung.:
# 'ws_1' (Orientierungscode Workspace 1)
# '1-l' (Zentrierungscode links (Workspace 1))
# '1-m' (Zentrierungscode Mitte (Workspace 1))
# '1-r' (Zentrierungscode rechts (Workspace 1))

class QrScan(Node):

    def __init__(self):
        super().__init__('qr_scan')
        self.publisher_ = self.create_publisher(QrPos, '/qr_pos', 1)
        self.state = True

        self.subscription_node_state = self.create_subscription(
            DockTrigger,
            'dock_trigger/qr_scan_trigger',
            self.listener_callback_manage_node_state,
            1)
        self.subscription_node_state

        self.cam_port = 0
        self.cam = None 

        self.timer_time = 0
        self.timer_iteration = 0

        if __name__ == '__main__':
            self.goal_ws_nr = 1
            self.start_node()

    def listener_callback_manage_node_state(self,msg):
        if msg.trigger:
            self.goal_ws_nr = msg.wsnumber
            self.start_node()
        else:
            self.stop_node()

    def start_node(self):
        self.state = True
        self.cam = cv2.VideoCapture(self.cam_port)
        self.timer = self.create_timer(0.1, self.start_scan)

    def stop_node(self):
        self.state = False
        self.cam = None
        try:
            self.timer.destroy()
        except:
            pass

        #Berechnet die durchschnittliche Frequenz der Node
        self.get_logger().info(f'qr scan avg hz: {1/(self.timer_time/self.timer_iteration)}')
        self.timer_time = 0
        self.timer_iteration = 0


    def callc_possition_t_c(self,points,mid_target,identifier):
        if points is not None:
            x1 = float(points[0][0])
            x2 = float(points[1][0])
            x3 = float(points[2][0])
            x4 = float(points[3][0])

            x_mean_r = (x2 + x3)/2
            x_mean_l = (x1 + x4)/2

            qr_mid_x = (x_mean_r - x_mean_l)/2 + x_mean_l

            if mid_target > qr_mid_x:
                dif = (mid_target - qr_mid_x)/mid_target
                return (mid_target,qr_mid_x,1.0,float(dif),identifier)#links
            else:
                dif = (mid_target - qr_mid_x)/mid_target
                return (mid_target,qr_mid_x,-1.0,float(dif),identifier)#rechts
        else:
            return (mid_target,None,0.0,0.0,identifier)
        
    def callc_possition_r_l(self,points,mid_target,identifier):
        if points is not None:
            x1 = float(points[0][0])
            x2 = float(points[1][0])
            x3 = float(points[2][0])
            x4 = float(points[3][0])

            x_mean_r = (x2 + x3)/2
            x_mean_l = (x1 + x4)/2

            qr_mid_x = (x_mean_r - x_mean_l)/2 + x_mean_l

            if identifier == 'l':
                direction = -1.0
            elif identifier == 'r':
                direction = 1.0
            
            if mid_target > qr_mid_x:
                dif = (mid_target - qr_mid_x)/mid_target
                return (mid_target,qr_mid_x,direction,float(dif),identifier)#links
            else:
                dif = (mid_target - qr_mid_x)/mid_target
                return (mid_target,qr_mid_x,direction,float(dif),identifier)#rechts
        else:
            return (mid_target,None,0.0,0.0,identifier)

        
    def scan(self,img):
        try:
            mid_target = int(img.shape[1]/2)#horizontaler Mittelpunkt

            detector = cv2.QRCodeDetector()
            retval, decoded_info, points, straight_qrcode = detector.detectAndDecodeMulti(img)

            #Wenn kein Code gefunden wurde wird None zurückgegeben
            if not retval:
                return None

            qr_left = f'{self.goal_ws_nr}-l'
            qr_rigth = f'{self.goal_ws_nr}-r'
            qr_mid = f'{self.goal_ws_nr}-m'

            qr_center = f'ws_{self.goal_ws_nr}'

            if qr_center in decoded_info:
                index = decoded_info.index(qr_center)
                return self.callc_possition_t_c(points[index],mid_target,'t')
    
            elif qr_mid in decoded_info:
                index = decoded_info.index(qr_mid)
                return self.callc_possition_t_c(points[index],mid_target,'c')

            elif qr_left in decoded_info:
                index = decoded_info.index(qr_left)
                return self.callc_possition_r_l(points[index],mid_target,'l')

            elif qr_rigth in decoded_info:
                index = decoded_info.index(qr_rigth)
                return self.callc_possition_r_l(points[index],mid_target,'r')

            #Es wurde kein passender Code gefunden
            else:
                return None
        except:
            pass

    def start_scan(self):
        start_timer_time = time.time()
        if self.cam:
            try: 
                result, image = self.cam.read()
                res = self.scan(image)

                msg = QrPos()

                if res is not None:
                    msg.offset = res[3]
                    msg.qrcode = res[4]
                    self.publisher_.publish(msg)

                else:
                    #Default-Massage wenn kein Code gefunden werden kann
                    msg.offset = 0.0
                    msg.qrcode = ''
                    self.publisher_.publish(msg)

            except:
                pass

        self.timer_time += time.time() - start_timer_time
        self.timer_iteration += 1


def main(args=None):
    rclpy.init(args=args)

    qr_scan = QrScan()

    rclpy.spin(qr_scan)
    
    qr_scan.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()