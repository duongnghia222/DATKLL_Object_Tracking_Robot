import cv2
import numpy as np
import math

CAMERA_DEVICE_ID = 0
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
screen_center_point = (IMAGE_WIDTH // 2, IMAGE_HEIGHT // 2)

hsv_min = np.array((0, 0, 0))
hsv_max = np.array((0, 0, 0))

colors = []
last_circle = (0, 0)
object_cnt = None
tracking_object = None
circle_updated = False


def on_mouse_click(event, x, y, flags, frame):
    global colors
    global prev_tracking_object
    global tracking_object
    if event == cv2.EVENT_LBUTTONUP:
        if tracking_object is None:
            tracking_object = (x, y)
        color_bgr = frame[y, x]
        color_rgb = tuple(reversed(color_bgr))
        color_hsv = rgb2hsv(color_rgb[0], color_rgb[1], color_rgb[2])
        colors.append(color_hsv)


# R, G, B values are [0, 255].
# Normally H value is [0, 359]. S, V values are [0, 1].
# However in opencv, H is [0,179], S, V values are [0, 255].
# Reference: https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html
def hsv2rgb(h, s, v):
    h = float(h) * 2
    s = float(s) / 255
    v = float(v) / 255
    h60 = h / 60.0
    h60f = math.floor(h60)
    hi = int(h60f) % 6
    f = h60 - h60f
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)
    r, g, b = 0, 0, 0
    if hi == 0:
        r, g, b = v, t, p
    elif hi == 1:
        r, g, b = q, v, p
    elif hi == 2:
        r, g, b = p, v, t
    elif hi == 3:
        r, g, b = p, q, v
    elif hi == 4:
        r, g, b = t, p, v
    elif hi == 5:
        r, g, b = v, p, q
    r, g, b = int(r * 255), int(g * 255), int(b * 255)
    return r, g, b


def rgb2hsv(r, g, b):
    r, g, b = r / 255.0, g / 255.0, b / 255.0
    mx = max(r, g, b)
    mn = min(r, g, b)
    df = mx - mn
    if mx == mn:
        h = 0
    elif mx == r:
        h = (60 * ((g - b) / df) + 360) % 360
    elif mx == g:
        h = (60 * ((b - r) / df) + 120) % 360
    elif mx == b:
        h = (60 * ((r - g) / df) + 240) % 360
    if mx == 0:
        s = 0
    else:
        s = df / mx
    v = mx

    h = int(h / 2)
    s = int(s * 255)
    v = int(v * 255)

    return h, s, v


def dist(x1, y1, x2, y2):
    return (x1 - x2) ** 2 + (y1 - y2) ** 2


# =========== my vars ========
middle_x = IMAGE_WIDTH // 2
scale = 0.3
middle_diff = 50
left_bound = middle_x - middle_diff
right_bound = middle_x + middle_diff
area_threshold = 5000
# ===========================

if __name__ == "__main__":
    try:
        # create video capture
        cap = cv2.VideoCapture(CAMERA_DEVICE_ID)

        # set resolution to 320x240 to reduce latency
        cap.set(3, IMAGE_WIDTH)
        cap.set(4, IMAGE_HEIGHT)
        while True:
            # Read the frames from a camera

            _, frame = cap.read()
            frame = cv2.flip(frame, 1)

            c_frame = frame.copy()
            c_frame = cv2.medianBlur(c_frame, 5)
            gray = cv2.cvtColor(c_frame, cv2.COLOR_BGR2GRAY)

            frame = cv2.GaussianBlur(frame, (3, 3), 0)
            cv2.namedWindow("frame")
            # Convert the image to hsv space and find range of colors
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            cv2.setMouseCallback('frame', on_mouse_click, frame)
            # find circle
            rows = gray.shape[0]
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1,
                                       minDist=rows / 8, param1=200, param2=37, minRadius=10, maxRadius=150)
            prev_circle = None
            if circles is not None:
                circles = np.uint16(np.around(circles))
                chosen = None
                for i in circles[0, :]:
                    if chosen is None:
                        chosen = i
                    if prev_circle is not None:
                        if dist(chosen[0], chosen[1], prev_circle[0], prev_circle[1]) <= dist(i[0], i[1],
                                                                                              prev_circle[0],
                                                                                              prev_circle[1]):
                            chosen = i
                    prev_circle = chosen
                cv2.circle(frame, (chosen[0], chosen[1]), 1, (0, 100, 100), 3)
                cv2.circle(frame, (chosen[0], chosen[1]), chosen[2], (0, 0, 255), 3)
                circle_updated = True
                if tracking_object is None:
                    tracking_object = (chosen[0], chosen[1])
                last_circle = (chosen[0], chosen[1])

            # find the color using a color threshold
            if colors:
                # find max & min h, s, v
                minh = min(c[0] for c in colors)
                mins = min(c[1] for c in colors)
                minv = min(c[2] for c in colors)
                maxh = max(c[0] for c in colors)
                maxs = max(c[1] for c in colors)
                maxv = max(c[2] for c in colors)

                # print("New HSV threshold: ", (minh, mins, minv), (maxh, maxs, maxv))
                hsv_min = np.array((minh, mins, minv))
                hsv_max = np.array((maxh, maxs, maxv))

            thresh = cv2.inRange(hsv, hsv_min, hsv_max)
            thresh2 = thresh.copy()
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            # finding contour with maximum area and store it as best_cnt
            contours_found = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 200:
                    peri = cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                    x, y, w, h = cv2.boundingRect(approx)
                    cx, cy = x + (w // 2), y + (h // 2)
                    contours_found.append(
                        {"cnt": cnt, "area": area, "bbox": [x, y, w, h], "center": [cx, cy], "vote": 0})
            contours_found = sorted(contours_found, key=lambda x: x["area"], reverse=True)
            print('contours found: ', len(contours_found))
            for i in range(3):  # find most voted biggest cnt
                if i < len(contours_found):
                    if circle_updated and dist(contours_found[i]['center'][0], contours_found[i]['center'][1],
                                               last_circle[0], last_circle[1]) < 200:
                        contours_found[i]['vote'] += 200
                    if tracking_object and dist(contours_found[i]['center'][0], contours_found[i]['center'][1],
                                                tracking_object[0], tracking_object[1]) < 5000:
                        contours_found[i]['vote'] += 75
                    if dist(contours_found[i]['center'][0], contours_found[i]['center'][1],
                            screen_center_point[0], screen_center_point[1]) < 2000:
                        contours_found[i]['vote'] += 25
                    contours_found[i]['vote'] += (2 - i)
            most_voted_cnt = -999
            for i in range(3):
                if i < len(contours_found):
                    if contours_found[i]['vote'] > most_voted_cnt:
                        most_voted_cnt = contours_found[i]['vote']
                        object_cnt = contours_found[i]
            if object_cnt is not None:
                # print(object_cnt['vote'])
                if contours_found:
                    cv2.circle(frame, tracking_object, 5, (0, 255, 255), -1)
                tracking_object = object_cnt['center']

            # finding centroids of best_cnt and draw a circle there
            if contours_found:
                # print('Area:', area)
                cx, cy = object_cnt['center']
                x, y, w, h = object_cnt['bbox']
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.circle(frame, tracking_object, 5, 255, -1)
                # print("Central pos: (%d, %d)" % (cx,cy))

                diff = abs(middle_x - cx)
                turn_speed = diff * scale
                middle_diff = int(object_cnt['area'] * 0.02)
                if middle_diff > 160:
                    middle_diff = 160
                left_bound = middle_x - middle_diff
                right_bound = middle_x + middle_diff
            #     if right_bound < cx:
            #         print("turn_right")
            #     elif left_bound > cx:
            #         print('turn left')
            #     else:
            #         if object_cnt['area'] < area_threshold:
            #             print('forward')
            #         if object_cnt['area'] > area_threshold + 5000:
            #             print('backward ')
            #         elif area_threshold <= object_cnt['area'] <= area_threshold + 5000:
            #             print('stop')
            # else:
            #     print('stop')

            # Show the original and processed image
            # res = cv2.bitwise_and(frame, frame, mask=thresh2)
            # cv2.line(frame, (middle_x, 540), (middle_x, 0), (255, 0, 0), thickness=1)
            # cv2.line(frame, (left_bound, 540), (left_bound, 0), (0, 255, 0), thickness=2)
            # cv2.line(frame, (right_bound, 540), (right_bound, 0), (0, 255, 0), thickness=2)
            cv2.imshow('frame', frame)
            cv2.imshow('thresh', thresh2)

            # if key pressed is 'Esc' then exit the loop
            if cv2.waitKey(33) == 27:
                break
            if cv2.waitKey(1) & 0xFF == ord('r'):  # press r to reset color
                colors = []
                print('reset color')
            circle_updated = False
    finally:
        # Clean up and exit the program
        cv2.destroyAllWindows()
        cap.release()
