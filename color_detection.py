import cv2
import numpy as np
import imutils
# color picker
cap = cv2.VideoCapture(0)
cap.set(3, 720) # width
cap.set(4, 1080) # height
cap.set(10, 100) # brightness

def empty(x):
    pass

cv2.namedWindow("trackBars")
cv2.resizeWindow("trackBars", 640, 240)
cv2.createTrackbar("Hue Min", "trackBars", 0, 179, empty)
cv2.createTrackbar("Hue Max", "trackBars", 15, 179, empty)
cv2.createTrackbar("Sat Min", "trackBars", 161, 255, empty)
cv2.createTrackbar("Sat Max", "trackBars", 255, 255, empty)
cv2.createTrackbar("Val Min", "trackBars", 108, 255, empty)
cv2.createTrackbar("Val Max", "trackBars", 255, 255, empty)
while True:
    _, img = cap.read()
    #cv2.imshow("WebCam", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # press q to quit
        break
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h_min = cv2.getTrackbarPos("Hue Min", "trackBars")
    h_max = cv2.getTrackbarPos("Hue Max", "trackBars")
    s_min = cv2.getTrackbarPos("Sat Min", "trackBars")
    s_max = cv2.getTrackbarPos("Sat Max", "trackBars")
    v_min = cv2.getTrackbarPos("Val Min", "trackBars")
    v_max = cv2.getTrackbarPos("Val Max", "trackBars")
    print(h_min,h_max,s_min,s_max,v_min,v_max)
    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])
    mask = cv2.inRange(imgHSV,lower,upper)
    imgRes = cv2.bitwise_and(img, img, mask=mask)
    #cv2.imshow("Original", img)
    #cv2.imshow("HSV", imgHSV)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", imgRes)
    cv2.waitKey(1)