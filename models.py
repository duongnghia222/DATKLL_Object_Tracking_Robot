import cv2
import numpy as np
import imutils


def hough_circle(img):
    def dist(x1, y1, x2, y2):
        return (x1 - x2) ** 2 * +(y1 - y2) ** 2

    prev_circle = None
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur2 = cv2.medianBlur(gray, 5)
    rows = gray.shape[0]
    circles = cv2.HoughCircles(blur2, cv2.HOUGH_GRADIENT, 1,
                               minDist=rows / 8, param1=210, param2=37, minRadius=20, maxRadius=150)
    print(circles)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        chosen = None
        for i in circles[0, :]:
            if chosen is None:
                chosen = i
            if prev_circle is not None:
                if dist(chosen[0], chosen[1], prev_circle[0], prev_circle[1]) <= dist(i[0], i[1], prev_circle[0],
                                                                                      prev_circle[1]):
                    chosen = i
            cv2.circle(img, (chosen[0], chosen[1]), 1, (0, 100, 100), 3)
            cv2.circle(img, (chosen[0], chosen[1]), chosen[2], (0, 0, 255), 3)
            prev_circle = chosen


def hsv_contour(img, lower_red, upper_red, r_threshold):
    blurred = cv2.GaussianBlur(img, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_red = cv2.erode(mask_red, None, iterations=2)
    mask_red = cv2.dilate(mask_red, None, iterations=2)
    cnts_red = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_red = imutils.grab_contours(cnts_red)
    if len(cnts_red) > 0:
        cnt_max = max(cnts_red, key=cv2.contourArea)
        # print(cnt_max)
        # print("-------------------")
        ((x, y), r) = cv2.minEnclosingCircle(cnt_max)
        if r > r_threshold:
            cv2.circle(img, (int(x), int(y)), int(r), (255, 0, 0), 2)
            cv2.circle(img, (int(x), int(y)), 1, (0, 255, 0), 2)
