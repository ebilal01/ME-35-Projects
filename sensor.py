import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
# Assign GPIO pin numbers to variables

s0 = 13
s1 = 15
s2 = 16
s3 = 18
sig = 22 #labeled "out" on your board
cycles = 10
OUT1 = 12 
# Define the GPIO pins for the L298N motor driver
OUT2 = 19
OUT3 = 21
OUT4 = 23
OUT5 = 24

# Set GPIO mode
GPIO.setmode(GPIO.BOARD)

SERVO_PIN = 36  # Using Pin 36 for the servo

# Setup Servo Motor
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz PWM frequency
servo.start(7.5)  # Neutral position (90 degrees)

def move_servo(angle):
    """Moves the servo to a specific angle (0° to 180°)."""
    duty = 2.5 + (angle / 18)  # Convert angle to duty cycle
    servo.ChangeDutyCycle(duty)
    time.sleep(0.5)  # Allow time to reach position

GPIO.setup(OUT2, GPIO.OUT)
GPIO.setup(OUT3, GPIO.OUT)
GPIO.setup(OUT4, GPIO.OUT)
GPIO.setup(OUT5, GPIO.OUT)

GPIO.output(OUT2, GPIO.LOW)
GPIO.output(OUT3, GPIO.LOW)
GPIO.output(OUT4, GPIO.LOW)
GPIO.output(OUT5, GPIO.LOW)

# Setup GPIO and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(OUT1, GPIO.OUT)
GPIO.setup(s0, GPIO.OUT)
GPIO.setup(s1, GPIO.OUT)
GPIO.setup(s2, GPIO.OUT)
GPIO.setup(s3, GPIO.OUT)
GPIO.setup(sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.output(OUT1, GPIO.HIGH)

# Set frequency scaling
GPIO.output(s0, GPIO.HIGH)
GPIO.output(s1, GPIO.LOW)

# Sets delay between steps
step_delay = 0.03
color = "test"
# Define the step sequence
step_sequence = [
    (GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW),
    (GPIO.LOW, GPIO.HIGH, GPIO.HIGH, GPIO.LOW),
    (GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH),
    (GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH),
]

def move_motor(steps, direction=1):
    """ Moves the stepper motor a given number of steps in a specified direction. """
    current_step = 0
    for _ in range(abs(steps)):
        GPIO.output(OUT2, step_sequence[current_step][0])
        GPIO.output(OUT3, step_sequence[current_step][1])
        GPIO.output(OUT4, step_sequence[current_step][2])
        GPIO.output(OUT5, step_sequence[current_step][3])
        time.sleep(step_delay)
        
        # Move step index in specified direction
        current_step = (current_step + direction) % 4

def servo_return():
    print("Moving clockwise (to 80 degrees)...")
    move_servo(80)
    time.sleep(1)

    print("Moving counterclockwise (to 25 degrees)...")
    move_servo(25)  # Move in the opposite direction
    time.sleep(2)

def turn(color):
    print("turning...")

    if(color == "red"):
        steps = 0
        move_motor(steps, direction=1)
    elif(color == "blue"):
        steps = 180
        move_motor(steps, direction=1)
    elif(color == "yellow"):
        steps = 90
        move_motor(steps, direction=1)
    elif(color == "green"):
        steps = 270
        move_motor(steps, direction=1)
    elif(color == "none"):
        print("no color detected")

def datum(color):
    print("returning...")
    if(color == "red"):
        steps = 0
        move_motor(steps, direction=-1)
    elif(color == "blue"):
        steps = 180
        move_motor(steps, direction=-1)
    elif(color == "yellow"):
        steps = 90
        move_motor(steps, direction=-1)
    elif(color == "green"):
        steps = 270
        move_motor(steps, direction=-1)
    elif(color == "none"):
        print("no color detected")
 
    


def DetectColor():

    
    # Detect red values
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.LOW)
    time.sleep(.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    red = cycles / duration
    print("red value - ", red)

    # Detect blue values
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    blue = cycles / duration
    print("blue value - ", blue)

    # Detect green values
    GPIO.output(s2, GPIO.HIGH)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    green = cycles / duration
    print("green value - ", green)

    if ( green >= 7000 and green <= 8000 and red >= 6000 and red <= 70000 and blue >= 8000 and blue <= 9000 ):
        print( "the color is blue")
        color = "blue"
        turn(color)
        servo_return()
        datum(color)
    elif ( green >= 7000 and green <= 8500 and red >= 8000 and red <= 9000 and blue >= 9000 and blue <= 10000 ):
        print( "the color is red")
        color = "red"
        turn(color)
        servo_return()
        datum(color)
    elif ( green >= 7000 and green <= 8500 and red >= 7000 and red <= 8000 and blue >= 9000 and blue <= 10000 ):
        print( "the color is green")
        color = "green"
        turn(color)
        servo_return()
        datum(color)
    elif ( green >= 9000 and green <= 10000 and red >= 10000 and red <= 11500 and blue >= 10000 and blue <= 11500 ):
        print( "the color is yellow")
        color = "yellow"
        turn(color)
        servo_return()
        datum(color)

    else:
        color = "none"
        turn(color)
    



    

def testColor():
    # Detect red values
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.LOW)
    time.sleep(.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    red = cycles / duration
    print("red value - ", red)

    # Detect blue values
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    blue = cycles / duration
    print("blue value - ", blue)

    # Detect green values
    GPIO.output(s2, GPIO.HIGH)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    green = cycles / duration
    print("green value - ", green)




def move():
    try:
        while True:
            print("moving to sensor (to 65 degrees)...")
            move_servo(55)
            time.sleep(5)

            DetectColor()


    except KeyboardInterrupt:
        print("\nStopping program...")
        servo.stop()
        GPIO.cleanup()




try:
    while True:
        move()


except KeyboardInterrupt:
    GPIO.cleanup()
