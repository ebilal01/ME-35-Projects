# Core opencv code provided by Einsteinium Studios
# Revisions to work with Pi Camera v3 by Briana Bouchard

import numpy as np
import cv2
from picamera2 import Picamera2
from libcamera import controls
import time
import RPi.GPIO as GPIO

picam2 = Picamera2() # assigns camera variable
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) # sets auto focus mode
picam2.start() # activates camera

sensor_res = picam2.sensor_resolution

time.sleep(1) # wait to give camera time to start up

GPIO.setmode(GPIO.BOARD)

# =================== MOTOR PINS ===================
motor_left_fwd = 29   # Left motor forward (BOARD Pin 29)
motor_left_bwd = 31   # Left motor backward (BOARD Pin 31)
motor_right_fwd = 33  # Right motor forward (BOARD Pin 33)
motor_right_bwd = 35  # Right motor backward (BOARD Pin 35)

GPIO.setup(motor_left_fwd, GPIO.OUT)
GPIO.setup(motor_left_bwd, GPIO.OUT)
GPIO.setup(motor_right_fwd, GPIO.OUT)
GPIO.setup(motor_right_bwd, GPIO.OUT)

# PWM setup for speed control
pin1 = GPIO.PWM(motor_left_fwd, 500)  # Left Forward PWM
pin2 = GPIO.PWM(motor_left_bwd, 500)  # Left Backward PWM
pin3 = GPIO.PWM(motor_right_fwd, 500)  # Right Forward PWM
pin4 = GPIO.PWM(motor_right_bwd, 500)  # Right Backward PWM

pin1.start(0)
pin2.start(0)
pin3.start(0)
pin4.start(0)

# =================== PID VARIABLES ===================
Kp = 0.1   # Proportional gain
Ki = 0    # Integral gain
Kd = 0   # Derivative gain

last_error = 0
integral = 0

def set_motor_speed(left_speed, right_speed):
    """Controls the motors using PWM speed and direction."""
    print(f"Left Speed: {left_speed}, Right Speed: {right_speed}")  # Debugging print

    # Set motor speeds for forward/backward movement, based on the input speeds
    if left_speed > 0:
        pin1.ChangeDutyCycle(abs(left_speed))  # Forward
        pin2.ChangeDutyCycle(0)
    else:
        pin1.ChangeDutyCycle(0)
        pin2.ChangeDutyCycle(abs(left_speed))  # Backward

    if right_speed > 0:
        pin3.ChangeDutyCycle(abs(right_speed))  # Forward
        pin4.ChangeDutyCycle(0)
    else:
        pin3.ChangeDutyCycle(0)
        pin4.ChangeDutyCycle(abs(right_speed))  # Backward

def pid_control(dir, error):
    """ Implements PID control for line following. """
    global last_error, integral, correction

    if (dir == 0): # going straight
        correction = 0
    if (dir == 1): # turn left
        correction = -10
    if (dir == -1): # turn right
        correction = 10

    # PID Calculation (for normal line following)
   
    proportional = Kp * error
    print(proportional)
    integral += Ki * error
    print(integral)
    derivative = Kd * (error - last_error)
    print(derivative)
   
    last_error = error

    # Combine correction and PID values
    correction += proportional + integral + derivative

    # Set motor speeds
    base_speed = 15  # Adjusted for slower speed to prevent overshooting
    left_motor_speed = abs(base_speed + correction)  # Ensuring a minimum speed
    right_motor_speed = abs(base_speed - correction)  # Ensuring a minimum speed

    set_motor_speed(left_motor_speed, right_motor_speed)

try:

    while True:

        # Display camera input
        image = picam2.capture_array("main")
        cv2.imshow('img',image)
   
        # Crop the image
        x_offset = 100
        y_offset = 50
        crop_img = image[60:120, 0:160]
        #crop_img = image[(sensor_res[0]//2)-x_offset:(sensor_res[0]//2)+x_offset, (sensor_res[1]//2)-y_offset:(sensor_res[1]//2)+y_offset]

        # Convert to grayscale
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
   
        # Gaussian blur
        blur = cv2.GaussianBlur(gray,(5,5),0)
   
        # Color thresholding
        input_threshold,comp_threshold = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV)
   
        # Find the contours of the frame
        contours,hierarchy = cv2.findContours(comp_threshold.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
   
        # Find the biggest contour (if detected)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c) # determine moment - weighted average of intensities

            if int(M['m00']) != 0:
                cx = int(M['m10']/M['m00']) # find x component of centroid location
                cy = int(M['m01']/M['m00']) # find y component of centroid location
            else:
                print("Centroid calculation error, looping to acquire new values")
                continue
            cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1) # display vertical line at x value of centroid
            cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1) # display horizontal line at y value of centroid
   
            cv2.drawContours(crop_img, contours, -1, (0,255,0), 2) # display green lines for all contours

            # determine location of centroid in x direction and adjust steering recommendation

            #set error value based on distance from the center of the frame
            print("cx: ", cx)
            error = abs(80 - cx)

            if cx >= 120:
                print("Turn Left!")
                pid_control(1, error)
   
            if cx < 120 and cx > 50:
                print("On Track!")
                pid_control(0, error)
   
            if cx <= 50:
                print("Turn Right")
                pid_control(-1, error)
   
        else:
            print("I don't see the line")
   
        # Display the resulting frame
        cv2.imshow('frame',crop_img)
       
        # Show image for 1 ms then continue to next image
        cv2.waitKey(1)

except KeyboardInterrupt:
    print('All done')
    GPIO.cleanup()
