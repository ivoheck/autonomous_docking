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

#Qr code benennung:
# 'ws_1' (orientierungs code workespace 1)
# '1-l' (centrirungs code links (workspace 1))
# '1-m' (centrirungs code mitte (workspace 1))
# '1-r' (centrirungs code rechts (workspace 1))

class QrScan(Node):

    def __init__(self):
        super().__init__('qr_scan')
        self.publisher_ = self.create_publisher(QrPos, '/qr_pos', 1)
        self.state = True

        self.subscription_node_state = self.create_subscription(
            DockTrigger,
            'trigger_dock_node/qr_scan_node',
            self.listener_callback_manage_node_state,
            1)
        self.subscription_node_state

        self.cam_port = 0
        self.cam = cv2.VideoCapture(self.cam_port)
        #720p qualität
        #self.cam.set(3, 1280)
        #self.cam.set(4, 720)

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
        #self.get_logger().info('start qr_scan_node')
        self.timer = self.create_timer(0.1, self.start_scan)

    def stop_node(self):
        self.state = False
        self.timer.destroy()

    def callc_movement(self,points,mid_target,img,identifier):
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
                #print('left',dif)
                return (mid_target,qr_mid_x,1.0,float(dif),identifier)#links
            else:
                #dif = (qr_mid_x- mid_target)/mid_target
                dif = (mid_target - qr_mid_x)/mid_target
                #print('rigth',dif)
                return (mid_target,qr_mid_x,-1.0,float(dif),identifier)#rechts
        else:
            return (mid_target,None,0.0,0.0,identifier) 
        
    def scan(self,img):
        try:
            mid_target = int(img.shape[1]/2)#horizontaler mittelpunkt

            detector = cv2.QRCodeDetector()
            retval, decoded_info, points, straight_qrcode = detector.detectAndDecodeMulti(img)
            #print(decoded_info)

            #Wenn kein code gefunden wrde wird None returned
            if not retval:
                return None

            qr_left = f'{self.goal_ws_nr}-l'
            qr_rigth = f'{self.goal_ws_nr}-r'
            qr_mid = f'{self.goal_ws_nr}-m'

            qr_center = f'ws_{self.goal_ws_nr}'

            if qr_center in decoded_info:
                index = decoded_info.index(qr_center)
                #TODO: hier schauen ob unterschied gemacht werden muss
                return self.callc_movement(points[index],mid_target,img,'t')
    
            elif qr_mid in decoded_info:
                index = decoded_info.index(qr_mid)
                return self.callc_movement(points[index],mid_target,img,'c')

            #TODO: evtl diesen case extra betrachten
            #elif qr_left in decoded_info and qr_rigth in decoded_info:
            #    pass
    
            elif qr_left in decoded_info:
                return (None,None,-1.0,1.0,'l')

            elif qr_rigth in decoded_info:
                return (None,None,1.0,1.0,'r')

            #Es wurde codes gefunden aber nicht die passenden dieser fall wird gewerte wie als wenn keiner gefunden wurde
            #da man jetzt nicht sicher stellen kann wo sich der roboter befindet
            else:
                return None
                #return (mid_target,None,0.0,0.0)
        except:
            pass

    def start_scan(self):
        if self.cam:
            try: 
                result, image = self.cam.read()
                res = self.scan(image)

                msg = QrPos()

                if res is not None:
                    msg.direction = res[2]
                    msg.offset = res[3]
                    msg.qrcode = res[4]
                    self.publisher_.publish(msg)

                else:
                    msg.direction = 0.0
                    msg.offset = 0.0
                    msg.qrcode = ''
                    self.publisher_.publish(msg)

            except:
                pass


def main(args=None):
    rclpy.init(args=args)

    qr_scan = QrScan()

    rclpy.spin(qr_scan)
    
    qr_scan.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()