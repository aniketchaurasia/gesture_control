import cv2 as cv
from utils import CvFpsCalc
from gestures import *
import socket



import threading
def main():
  gesture_detector = GestureRecognition(use_static_image_mode, min_detection_confidence,min_tracking_confidence)
  #gesture_buffer = GestureBuffer(buffer_len=buffer_len)
  cv_fps_calc = CvFpsCalc(buffer_len=10)
  mode = 0
  number = -1
  # Camera capture
  HOST = "192.168.43.99"  # The server's hostname or IP address
  PORT = 65432  # The port used by the server"""
  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
      s.connect((HOST, PORT))
      cap = cv.VideoCapture(0)
      while cap.isOpened():
        success, image = cap.read()
        if not success:
          continue
        fps = cv_fps_calc.get()
        key = cv.waitKey(1) & 0xff
        if key == 27:  # ESC
          messageend="esc"
          s.send(messageend.encode())
          break
        #image = cap.frame
        elif key == 32: #space
          messageend="space"
          s.send(messageend.encode())
          continue

        debug_image, gesture_id = gesture_detector.recognize(image, number, mode)
        # echo-client.py
        gesture_id=str(gesture_id)
        s.send(gesture_id.encode())
        #print(type(gesture_id),gesture_id)
        #gesture_buffer.add_gesture(gesture_id)
        #print(gesture_id)
        debug_image = gesture_detector.draw_info(debug_image, fps, mode, number)

        cv.imshow('Gesture Recognition', debug_image)

      cv.destroyAllWindows()

use_static_image_mode=False
min_detection_confidence=0.7
min_tracking_confidence=0.5
buffer_len=5
main()