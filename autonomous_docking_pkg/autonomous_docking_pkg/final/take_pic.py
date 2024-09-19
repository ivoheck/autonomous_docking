from cv2 import *
import cv2

def take_pic():
    cam_port = 0
    cam = cv2.VideoCapture(cam_port)
    cam.set(3, 1920)
    cam.set(4, 1080)
    result, image = cam.read()
    cv2.imwrite("test.png", image)

take_pic()
