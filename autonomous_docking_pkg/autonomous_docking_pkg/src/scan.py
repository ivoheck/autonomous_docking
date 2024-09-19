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
        x1 = float(bbox[0][0][0])
        x2 = float(bbox[0][1][0])
        x3 = float(bbox[0][2][0])
        x4 = float(bbox[0][3][0])

        x_mean_r = (x2 + x3)/2
        x_mean_l = (x1 + x4)/2

        qr_mid_x = (x_mean_r - x_mean_l)/2 + x_mean_l

        if mid_target > qr_mid_x:
            dif = (mid_target - qr_mid_x)/(img.shape[1]/2)
            return (mid_target,qr_mid_x,1.0,float(dif))#links
        else:
            dif = (qr_mid_x- mid_target)/(img.shape[1]/2)
            return (mid_target,qr_mid_x,-1.0,float(dif))#rechts
    else:
        return (mid_target,None,0.0,0.0)


class QrScan(Node):

    def __init__(self):
        super().__init__('qr_scan')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/qr_pos', 10)
        self.state = True
        self.start_scan()

    def start_scan(self):
        cam_port = 0
        cam = VideoCapture(cam_port) 
        if cam: 
            while self.state:
                time_ = time.time()
                result, image = cam.read() 
                if result:
                    res = scan(image)

                    msg = Float32MultiArray()
                    msg.data = [res[2],res[3]]
                    self.publisher_.publish(msg)
                    print(res)
                else:
                    print('error with image taking')
                print(time.time() - time_)
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

  
 
  

  
