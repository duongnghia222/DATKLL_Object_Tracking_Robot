import cvzone
import cv2
from cvzone.ColorModule import ColorFinder
import RPi.GPIO as GPIO
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
    
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
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
camera.brightness = 60
# end camera config ============

# =========== my vars ========
middle_x = 330
scale = 0.3
middle_diff = 50
left_bound = middle_x - middle_diff
right_bound = middle_x + middle_diff
area_threshold = 23000
# ===========================
keyboard_control = False


# ===========================
myColorFinder = ColorFinder(False)  # change this to True to get the color you want
hsvVals = {'hmin': 163, 'smin': 0, 'vmin': 0, 'hmax': 179, 'smax': 255, 'vmax': 255}
#hsvVals = {'hmin': 28, 'smin': 75, 'vmin': 0, 'hmax': 71, 'smax': 254, 'vmax': 255}
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
            imgColor, mask = myColorFinder.update(img, hsvVals)
            imgContour, contours = cvzone.findContours(img,mask, minArea= 2000)
            if contours:
                data = contours[0]['center'][0],\
                        contours[0]['center'][1],\
                        int(contours[0]['area'])
    #             print(data)
    #             print(right_bound, left_bound)
                
                diff = abs(middle_x - data[0])
                turn_speed = diff*scale
                middle_diff = int(data[2]*0.02)
                if(middle_diff > 160):
                    middle_diff = 160
                print('middle', middle_diff)
                left_bound = middle_x - middle_diff
                right_bound = middle_x + middle_diff
                if right_bound < data[0]:
                    turn_right(init_default_speed)
                elif left_bound > data[0]:
                    turn_left(init_default_speed)
                else:
                    if data[2] < area_threshold:
                        forward(init_default_speed)
                    if data[2] > area_threshold + 8000:
                        backward(init_default_speed)
                    elif area_threshold <= data[2] <= area_threshold + 5000:
                        stop()
            else:
                stop()
                
            cv2.line(imgContour, (middle_x,540),  (middle_x,0) ,(255,0,0), thickness = 1)
            cv2.line(imgContour, (left_bound, 540), (left_bound, 0), (0, 255, 0), thickness=2)
            cv2.line(imgContour, (right_bound, 540), (right_bound, 0), (0, 255, 0), thickness=2)
            imgStack = cvzone.stackImages([img, imgColor, mask, imgContour], 2, 0.5)
            cv2.imshow("Result", imgStack)
            #cv2.imshow("Result", imgContour)
            #cv2.imshow("Frame", img)
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
        





