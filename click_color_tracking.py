import cv2
import numpy as np
import math

CAMERA_DEVICE_ID = 0
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

hsv_min = np.array((50, 80, 80))
hsv_max = np.array((120, 255, 255))

colors = []
last_circle = None
prev_tracking_object = None
tracking_object = None
object_cnt = None


def on_mouse_click(event, x, y, flags, frame):
    global colors
    global prev_tracking_object
    global tracking_object
    if event == cv2.EVENT_LBUTTONUP:
        if prev_tracking_object is None and tracking_object is None:
            tracking_object = (x, y)
            prev_tracking_object = (x, y)
        color_bgr = frame[y, x]
        color_rgb = tuple(reversed(color_bgr))
        color_hsv = rgb2hsv(color_rgb[0], color_rgb[1], color_rgb[2])
        if color_hsv in colors:
            colors.remove(color_hsv)
        else:
            colors.append(color_hsv)
        print(colors)


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
                                       minDist=rows / 8, param1=210, param2=37, minRadius=10, maxRadius=150)
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
                    cv2.circle(frame, (chosen[0], chosen[1]), 1, (0, 100, 100), 3)
                    cv2.circle(frame, (chosen[0], chosen[1]), chosen[2], (0, 0, 255), 3)
                    last_circle = chosen
                    prev_circle = chosen
            # print('last found circle', last_circle)
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
                if area > 700:
                    peri = cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                    x, y, w, h = cv2.boundingRect(approx)
                    cx, cy = x + (w // 2), y + (h // 2)
                    contours_found.append({"cnt": cnt, "area": area, "bbox": [x, y, w, h], "center": [cx, cy]})
            contours_found = sorted(contours_found, key=lambda x: x["area"], reverse=True)
            if contours_found:
                object_cnt = contours_found[0]['cnt']
            for i in range(3):  # find 3 biggest cnt
                if i < len(contours_found):
                    if dist(contours_found[i]['center'][0], contours_found[i]['center'][1], last_circle[0], \
                            last_circle[1]) < 100 or dist(contours_found[i]['center'][0],
                                                          contours_found[i]['center'][1], prev_tracking_object[0], \
                                                          prev_tracking_object[1]) < 7000:
                        prev_tracking_object = (tracking_object[0], tracking_object[1])
                        tracking_object = (contours_found[i]['center'][0], contours_found[i]['center'][1],
                                           contours_found[i]['area'])
                        object_cnt = contours_found[i]['cnt']
                        break

                    # if init_object_center:
                    #     M = cv2.moments(cnt)
                    #     cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                    #     if prev_tracking_object is None:
                    #         prev_tracking_object = init_object_center
                    #         tracking_object = (init_object_center[0], init_object_center[1], 0)
                    #     print('cx cy prev')
                    #     print(cx, cy, prev_tracking_object)
                    #     print(dist(cx, cy, prev_tracking_object[0], prev_tracking_object[1]))
                    #     print('cx cy last circle')
                    #     print(cx, cy, last_circle)
                    #     print(dist(cx, cy, last_circle[0], last_circle[1]))
                    #     if dist(cx, cy, prev_tracking_object[0], prev_tracking_object[1]) < 9000 or \
                    #             dist(cx, cy, last_circle[0], last_circle[1]) < 100:
                    #         prev_tracking_object = (tracking_object[0], tracking_object[1])
                    #         tracking_object = (cx, cy, area)
                    #         object_cnt = cnt
                    #         max_area = area

            # finding centroids of best_cnt and draw a circle there
            if object_cnt is not None:
                # print('Area:', area)
                cx, cy = tracking_object[0], tracking_object[1]

                peri = cv2.arcLength(object_cnt, True)
                approx = cv2.approxPolyDP(object_cnt, 0.02 * peri, True)
                x, y, w, h = cv2.boundingRect(approx)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.circle(frame, prev_tracking_object, 5, (0, 255, 255), -1)
                cv2.circle(frame, (cx, cy), 5, 255, -1)
                # print("Central pos: (%d, %d)" % (cx,cy))
            else:
                # print("[Warning]Tag lost...")
                pass

            # Show the original and processed image
            # res = cv2.bitwise_and(frame, frame, mask=thresh2)
            cv2.imshow('frame', frame)
            cv2.imshow('thresh', thresh2)

            # if key pressed is 'Esc' then exit the loop
            if cv2.waitKey(33) == 27:
                break
            if cv2.waitKey(1) & 0xFF == ord('r'):  # press r to reset color
                colors = []
                print('reset color')
    finally:
        # Clean up and exit the program
        cv2.destroyAllWindows()
        cap.release()
