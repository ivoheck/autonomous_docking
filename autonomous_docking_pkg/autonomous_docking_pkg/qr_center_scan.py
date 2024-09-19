#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://www.geeksforgeeks.org/how-to-capture-a-image-from-webcam-in-python/
#https://www.geeksforgeeks.org/webcam-qr-code-scanner-using-opencv/
#https://www.geeksforgeeks.org/reading-images-in-python/
#https://note.nkmk.me/en/python-opencv-qrcode/

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
#from cv2 import * 
import imageio
import time


#img = imageio.imread("qr_code.jpg")

#Qr code benennung:
# 'ws-1' (orientierungs code workespace 1)
# '1-l' (centrirungs code links (workspace 1))
# '1-m' (centrirungs code mitte (workspace 1))
# '1-r' (centrirungs code rechts (workspace 1))

#TODO: das müsste natürlich dynamisch mit unterschiedlichen nummern funktioniren
ws_nr = 1 #Nummer des aktuellen arbeitsplatzes

def scan(img):
    mid_target = int(img.shape[1]/2)#horizontaler mittelpunkt

    detector = cv2.QRCodeDetector()
    retval, decoded_info, points, straight_qrcode = detector.detectAndDecodeMulti(img)
    print(decoded_info)

    #Wenn kein code gefunden wrde wird None returned
    if not retval:
        return None

    qr_left = f'{ws_nr}-l'
    qr_rigth = f'{ws_nr}-r'
    qr_mid = f'{ws_nr}-m'

    qr_center = f'center'

    if qr_center in decoded_info:
        index = decoded_info.index(qr_center)
        #TODO: hier schauen ob unterschied gemacht werden muss
        return callc_movement(points[index],mid_target,img)
    
    elif qr_mid in decoded_info:
        index = decoded_info.index(qr_mid)
        return callc_movement(points[index],mid_target,img)

    #TODO: evtl diesen case extra betrachten
    #elif qr_left in decoded_info and qr_rigth in decoded_info:
    #    pass
    
    elif qr_left in decoded_info:
        return (None,None,-1.0,1.0)

    elif qr_rigth in decoded_info:
        return (None,None,1.0,1.0)

    #Es wurde codes gefunden aber nicht die passenden dieser fall wird gewerte wie als wenn keiner gefunden wurde
    #da man jetzt nicht sicher stellen kann wo sich der roboter befindet
    else:
        return None
        #return (mid_target,None,0.0,0.0)


def callc_movement(points,mid_target,img):
    if points is not None:
        x1 = float(points[0][0])
        x2 = float(points[1][0])
        x3 = float(points[2][0])
        x4 = float(points[3][0])

        x_mean_r = (x2 + x3)/2
        x_mean_l = (x1 + x4)/2

        qr_mid_x = (x_mean_r - x_mean_l)/2 + x_mean_l

        if mid_target > qr_mid_x:
            dif = (mid_target - qr_mid_x)/(img.shape[1]/2)
            print('left',dif)
            return (mid_target,qr_mid_x,1.0,float(dif))#links
        else:
            dif = (qr_mid_x- mid_target)/(img.shape[1]/2)
            print('rigth',dif)
            return (mid_target,qr_mid_x,-1.0,float(dif))#rechts
    else:
        return (mid_target,None,0.0,0.0)


class QrScan(Node):

    def __init__(self):
        super().__init__('qr_scan')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/qr_pos', 1)
        self.state = True
        self.start_scan()

    def start_scan(self):
        cam_port = 0
        cam = cv2.VideoCapture(cam_port)
        #1080p qualität
        #cam.set(3, 1920)
        #cam.set(4, 1080)
        if cam:
            while self.state:
                try: 
                    result, image = cam.read()
                    res = scan(image)

                    msg = Float32MultiArray()

                    if res is not None:
                        msg.data = [res[2],res[3]]
                        self.publisher_.publish(msg)

                    else:
                        msg.data = [0.0,0.0]
                        self.publisher_.publish(msg)

                except KeyboardInterrupt:
                    self.state = False
                    rclpy.shutdown()
        else: 
            print("No camera on current device") 



def main(args=None):
    rclpy.init(args=args)

    qr_scan = QrScan()

    rclpy.spin(qr_scan)
    
    qr_scan.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()