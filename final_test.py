import cvzone
import cv2
from cvzone.ColorModule import ColorFinder
import RPi.GPIO as GPIO
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import numpy as np
    
# ===== motor config ==============
in1 = 22
in2 = 24
in3 = 17
in4 = 27
enA = 6
enB = 26
servoPin = 13

init_default_speed = 30

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(servoPin, GPIO.OUT)
GPIO.setup(in1,GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(in2,GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(in3,GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(in4,GPIO.OUT, initial = GPIO.LOW)

GPIO.setup(enA,GPIO.OUT)
GPIO.setup(enB,GPIO.OUT)

pA = GPIO.PWM(enA,10)
pA.start(0)
pB = GPIO.PWM(enB,10)
pB.start(0)
servo = GPIO.PWM(servoPin, 50)
servo.start(0)



# ===== end motor config ============

# ===== dicrection config =============
def stop():
    pA.ChangeDutyCycle(0)
    pB.ChangeDutyCycle(0)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    
def backward(speed):
    pA.ChangeDutyCycle(speed)
    pB.ChangeDutyCycle(speed)
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    
def turn_right(speed):
    pA.ChangeDutyCycle(speed)
    pB.ChangeDutyCycle(speed)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    

def forward(speed):
    pA.ChangeDutyCycle(speed)
    pB.ChangeDutyCycle(speed)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
    
def turn_left(speed):
    pA.ChangeDutyCycle(speed)
    pB.ChangeDutyCycle(speed)
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
    
    

init_duty = 2
def servo_turn_left():
    global init_duty
    
    init_duty += 2
    if init_duty > 12:
        init_duty = 12
    servo.ChangeDutyCycle(init_duty)
    time.sleep(0.3)
    servo.ChangeDutyCycle(0)
    time.sleep(0.7)

def servo_turn_right():
    global init_duty
    
    init_duty -= 2
    if init_duty < 2:
        init_duty = 2
    servo.ChangeDutyCycle(init_duty)
    time.sleep(0.3)
    servo.ChangeDutyCycle(0)
    time.sleep(0.7)

def setAngle():
    servo.ChangeDutyCycle(2+(90/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)  



# ====== end direction config ==========

# camera config ==================
camera = PiCamera()
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
screen_center_point = (IMAGE_WIDTH // 2, IMAGE_HEIGHT // 2)
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
camera.brightness = 60
# end camera config ============

# =========== my vars ========
middle_x = IMAGE_WIDTH // 2
scale = 0.3
middle_diff = 50
left_bound = middle_x - middle_diff
right_bound = middle_x + middle_diff
area_threshold = 23000
# ===========================
last_circle = (0, 0)
object_cnt = None
tracking_object = None
circle_updated = False
keyboard_control = False
# ==========================

# ===========================
hsv_min = np.array((0, 0, 0))
hsv_max = np.array((0, 0, 0))
colors = []
# ===========================
# function begin
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


for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    try:
        if keyboard_control:
            cv2.imshow("Result", frame.array)
            key = cv2.waitKey(1) & 0xFF
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
            if key == ord("w"):
                forward(init_default_speed)
            if key == ord("a"):
                turn_left(17)
            if key == ord("d"):
                turn_right(17)
            if key == ord("s"):
                backward(int(init_default_speed*0.8))
            if key == ord("z"):
                servo_turn_left()
            if key == ord("c"):
                servo_turn_right()
            if key == ord("v"):
                setAngle()
            if key == ord("x"):
                stop()
            if key == ord("k"):
                stop()
                keyboard_control = not keyboard_control
                print("****** tracking car mode  *******")
        else:
            img = frame.array
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
                cv2.circle(frame, tracking_object, 5, (0, 255, 255), -1)
                tracking_object = object_cnt['center']
            if contours_found:
                # print('Area:', area)
                cx, cy = object_cnt['center']
                x, y, w, h = object_cnt['bbox']
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.circle(frame, tracking_object, 5, 255, -1)
                diff = abs(middle_x - cx)
                turn_speed = diff*scale
                middle_diff = int(object_cnt['area']*0.02)
                if middle_diff > 160:
                    middle_diff = 160
                left_bound = middle_x - middle_diff
                right_bound = middle_x + middle_diff
                if right_bound < cx:
                    turn_right(init_default_speed)
                elif left_bound > cx:
                    turn_left(init_default_speed)
                else:
                    if object_cnt['area'] < area_threshold:
                        forward(init_default_speed)
                    if object_cnt['area'] > area_threshold + 8000:
                        backward(init_default_speed)
                    elif area_threshold <= object_cnt['area'] <= area_threshold + 5000:
                        stop()
            else:
                stop()

            cv2.line(frame, (middle_x, 540), (middle_x, 0), (255, 0, 0), thickness=1)
            cv2.line(frame, (left_bound, 540), (left_bound, 0), (0, 255, 0), thickness=2)
            cv2.line(frame, (right_bound, 540), (right_bound, 0), (0, 255, 0), thickness=2)
            cv2.imshow('frame', frame)
            cv2.imshow('thresh', thresh2)
            key = cv2.waitKey(1) & 0xFF
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break
            if key == ord("i"):
                Thread(target=servo_turn_left).start()
            if key == ord("p"):
                Thread(target=servo_turn_right).start()
            if key == ord("o"):
                Thread(target=setAngle).start()    
            
            if key == ord("x"):
                stop()
            if key == ord("a"):
                turn_left(init_default_speed)
            if key == ord("d"):
                turn_right(init_default_speed)
            if key == ord("s"):
                backward(init_default_speed)
            if key == ord("w"):
                forward(init_default_speed)
            if key == ord("k"):
                stop()
                keyboard_control = not keyboard_control
                print("****** control car by keyboard  *******")
        if key == ord("r"):
                if init_default_speed <= 94:
                    init_default_speed += 5
                    print("speed = ", init_default_speed)
        if key == ord("f"):
            if init_default_speed >= 6:
                init_default_speed -= 5
                print("speed = ", init_default_speed)
    except:
        GPIO.cleanup()
        stop()
        





