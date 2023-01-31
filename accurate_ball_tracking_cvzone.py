import cvzone
import cv2
from cvzone.ColorModule import ColorFinder
import pygame

cap = cv2.VideoCapture(0)
cap.set(3, 720)  # width
cap.set(4, 1080)  # height
cap.set(10, 100)  # brightness
myColorFinder = ColorFinder(True)  # change this to True to get the color you want
# hsvVals = {'hmin': 163, 'smin': 122, 'vmin': 113, 'hmax': 179, 'smax': 255, 'vmax': 255}
hsvVals = {'hmin': 163, 'smin': 122, 'vmin': 113, 'hmax': 179, 'smax': 255, 'vmax': 255}
#
middle_x = 330
scale = 1
middle_diff = 30
left_bound = middle_x - middle_diff
right_bound = middle_x + middle_diff
area_threshold = 10000
#
while True:
    success, img = cap.read()
    imgColor, mask = myColorFinder.update(img, hsvVals)
    imgContour, contours = cvzone.findContours(img, mask, minArea= 1000)
    if contours:
        data = contours[0]['center'][0],\
                contours[0]['center'][1],\
                int(contours[0]['area'])
        print(data)
        diff = abs(middle_x - data[0])
        turn_speed = diff*scale
        if right_bound < data[0]:
            print('turn left')
        elif left_bound > data[0]:
            print('turn right')
        else:
            if data[2] < area_threshold:
                print('forward')
            if data[2] > area_threshold + 5000:
                print('backward')
            elif area_threshold <= data[2] <= area_threshold + 5000:
                print("stay")


    cv2.line(imgContour, (middle_x,540),  (middle_x,0) ,(255,0,0), thickness = 1)
    cv2.line(imgContour, (left_bound, 540), (left_bound, 0), (0, 255, 0), thickness=2)
    cv2.line(imgContour, (right_bound, 540), (right_bound, 0), (0, 255, 0), thickness=2)
    imgStack = cvzone.stackImages([img, imgColor, mask, imgContour], 2, 0.5)
    cv2.imshow("Result", imgStack)
    # cv2.imshow("Result", imgContour)

    if cv2.waitKey(1) & 0xFF == ord('q'):  # press q to quit
        break

