# 1. import library for computer vision
import cv2
import numpy as np
import imutils
import time
from models import hsv_contour, hough_circle
# ============================


# 2. define variable/const
# lower_red = np.array([0,161,108]) # my ball color
# upper_red = np.array([15,255,255]) # my ball color
lower_red = np.array([0, 100, 100])  # recommend red
upper_red = np.array([7, 255, 255])  # recommend red
# #green color
# lower_green = np.array([40,70,80])
# upper_green = np.array([70,255,255])
#
# #blue color
# lower_blue = np.array([90,60,0])
# upper_blue = np.array([121,255,255])
area_threshold = 100  # arg for an accurate detection
r_threshold = 30

# ==============================


# 3. webcam setup
cap = cv2.VideoCapture(0)
cap.set(3, 720)  # width
cap.set(4, 1080)  # height
cap.set(10, 100)  # brightness
time.sleep(2)  # time for camera warnup
# =================================

# loop for frame processing
while True:
    success, img = cap.read()
    hsv_contour(img, lower_red, upper_red, r_threshold)
    hough_circle(img)
    cv2.imshow("Result", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # press q to quit
        break


# end
print("the end")
