#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://www.geeksforgeeks.org/how-to-capture-a-image-from-webcam-in-python/
#https://www.geeksforgeeks.org/webcam-qr-code-scanner-using-opencv/
#https://www.geeksforgeeks.org/reading-images-in-python/

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2 as cv
from cv2 import * 
import imageio
import time
 
#img = imageio.imread("qr_code.jpg")


def scan(img):
    mid_target = int(img.shape[1]/2)#horizontaler mittelpunkt

    detector = cv.QRCodeDetector()
    data, bbox, _ = detector.detectAndDecode(img)
   
    if bbox is not None:
        #print(bbox[0])
        x1 = float(bbox[0][0][0])
        x2 = float(bbox[0][1][0])
        x3 = float(bbox[0][2][0])
        x4 = float(bbox[0][3][0])

        y1 = float(bbox[0][0][1])
        y2 = float(bbox[0][1][1])
        y3 = float(bbox[0][2][1])
        y4 = float(bbox[0][3][1])

        r_side = y2 - y1
        l_side = y3 - y4
        #15 = 50cm
        #5 = 70cm
        #y = -2x +80
        print(l_side)
        print((-2*l_side) +80)

        return (-2*l_side + 80, -2*r_side + 80)
    else:
        return (0.0,0.0)


class QrScan(Node):

    def __init__(self):
        super().__init__('qr_scan')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/relative_pos', 1)
        self.state = True
        self.start_scan()

    def start_scan(self):
        cam_port = 0
        cam = VideoCapture(cam_port) 
        if cam:
            while self.state:
                try:
                    result, image = cam.read() 
                    if result:
                        res_1 = scan(image)

                    result, image = cam.read() 
                    if result:
                        res_2 = scan(image)

                    result, image = cam.read() 
                    if result:
                        res_3 = scan(image)
                    #Wenn nur bei einem von drei werten ein qr code gefunden wird wird dieser ingnoriert
                    if res_1[0] == 0.0 and res_2[0] == 0.0 or res_1[0] == 0.0 and res_3[0] == 0.0 or res_2[0] == 0.0 and res_3[0] == 0.0: 
                        msg = Float32MultiArray()
                        msg.data = [0.0,0.0]
                        self.publisher_.publish(msg)
                        print(msg)

                    #TODO: Evtl schauen wie viel links und wie viele rechts sagen und dann aus dem gewinner den durchnit bilden
                    else:
                        msg = Float32MultiArray()
                        if res_1[0] != 0.0:
                            msg.data = [res_1[0],res_1[1]]
                        elif res_2[0] != 0.0:
                            msg.data = [res_2[0],res_2[1]]
                        else:
                            msg.data = [res_3[0],res_3[1]]
                        self.publisher_.publish(msg)
                        #print(msg)

                    #else:
                    #    print('error with image taking')
                except KeyboardInterrupt:
                    rclpy.shutdown()
        else: 
            print("No camera on current device") 



def main(args=None):
    rclpy.init(args=args)

    qr_scan = QrScan()

    rclpy.spin(qr_scan)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    qr_scan.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

  
 
  

  
