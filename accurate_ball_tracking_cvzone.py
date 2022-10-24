import cvzone
import cv2
from cvzone.ColorModule import ColorFinder


cap = cv2.VideoCapture(0)
cap.set(3, 720)  # width
cap.set(4, 1080)  # height
cap.set(10, 100)  # brightness
myColorFinder = ColorFinder(False)  # change this to True to get the color you want
hsvVals = {'hmin': 163, 'smin': 122, 'vmin': 113, 'hmax': 179, 'smax': 255, 'vmax': 255}
while True:
    success, img = cap.read()
    imgColor, mask = myColorFinder.update(img, hsvVals)
    imgContour, contours = cvzone.findContours(img,mask, minArea= 1000)
    if contours:
        data = contours[0]['center'][0],\
                contours[0]['center'][1],\
                int(contours[0]['area'])
        print(data)
    imgStack = cvzone.stackImages([img, imgColor, mask, imgContour], 2, 0.5)
    cv2.imshow("Result", imgStack)

    if cv2.waitKey(1) & 0xFF == ord('q'):  # press q to quit
        break

