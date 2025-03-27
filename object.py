import cv2
import time
import rclpy
import numpy as np
from rclpy.node import Node
from libcamera import controls
from PIL import Image, ImageOps
from picamera2 import Picamera2
from keras.models import load_model
from gpiozero import DistanceSensor
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from irobot_create_msgs.action import DriveDistance, RotateAngle
INCHES_CONV = 39.370
POKEBALL = "poke"
CHARIZARD = "zard"
BULBALSAUR = "bulba"
SQUIRTLE = "squirt"
PIKACHU = "pika"
NIODRAN = "nida"
SNORLAX = "snor"
# Initialize ROS 2
rclpy.init()
node = rclpy.create_node('drive_robot')
ultrasonic = DistanceSensor(echo=18, trigger=15, threshold_distance=0.25)
model = load_model("keras_model.h5", compile=False)
class_names = open("labels.txt", "r").readlines()
# Create action clients
client_drive = ActionClient(node, DriveDistance, '/drive_distance')
client_rotate = ActionClient(node, RotateAngle, '/rotate_angle')
# Track active goals
active_goals = []
picam2 = Picamera2()
# Configure the picamera
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})  # Set autofocus mode
picam2.start()  # Must start the camera before taking images
time.sleep(1)
def object_recognition(samples=10):
    recognition_dict = {}  # Dictionary to store class name and summed confidence scores
    for _ in range(samples):
        image = picam2.capture_array()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_AREA)
        image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)
        image = (image / 127.5) - 1  # Normalize between -1 and 1
        # Predict using the model
        prediction = model.predict(image)
        index = np.argmax(prediction)
        class_name = class_names[index].strip()
        confidence_score = prediction[0][index]
        print(f"Class: {class_name}, Confidence Score: {confidence_score * 100:.2f}%")
        # Update recognition dictionary
        if class_name in recognition_dict:
            recognition_dict[class_name] += confidence_score
        else:
            recognition_dict[class_name] = confidence_score
    # Determine the most recognized object
    object_recognized = max(recognition_dict, key=recognition_dict.get)
    max_confidence = recognition_dict[object_recognized]
    print(f"Final Recognition: {object_recognized}, Confidence: {max_confidence * 100:.2f}%")
    return object_recognized, max_confidence
def move_forward():
    print("Moving forward")
    client_drive.wait_for_server()
    goal_msg = DriveDistance.Goal()
    goal_msg.distance = 0.0127 * 5  # meters
    goal_msg.max_translation_speed = 0.01 * 5  # meters per second
    future = client_drive.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)
    if future.result():
        active_goals.append(future.result())  # Track goal handle
    print("Move complete.")
def stop_robot():
    print("STOPPING")
    client_drive.wait_for_server()
    goal_msg = DriveDistance.Goal()
    goal_msg.distance = 0.00001  # meters
    goal_msg.max_translation_speed = 0.0001  # meters per second
    future = client_drive.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)
    if future.result():
        active_goals.append(future.result())
    print("Stop complete.")
def rotate_left():
    print("Turning left")
    client_rotate.wait_for_server()
    rotate_goal = RotateAngle.Goal()
    rotate_goal.angle = -1.57  # radians (90 degrees left)
    rotate_goal.max_rotation_speed = 0.5  # radians per second
    future = client_rotate.send_goal_async(rotate_goal)
    rclpy.spin_until_future_complete(node, future)
    if future.result():
        active_goals.append(future.result())
    time.sleep(4)
    print("Turn complete.")
def rotate_right():
    print("Turning right")
    client_rotate.wait_for_server()
    rotate_goal = RotateAngle.Goal()
    rotate_goal.angle = 1.57  # radians (90 degrees right)
    rotate_goal.max_rotation_speed = 0.5  # radians per second
    future = client_rotate.send_goal_async(rotate_goal)
    rclpy.spin_until_future_complete(node, future)
    if future.result():
        active_goals.append(future.result())
    time.sleep(4)
    print("Turn complete.")
def get_proximity():
    return ultrasonic.distance * INCHES_CONV
def clear_all_actions():
    global active_goals
    print("Clearing all active actions...")
    # Cancel all tracked goals
    for goal_handle in active_goals:
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future)
        if cancel_future.result():
            print(f"Goal {goal_handle} canceled.")
    active_goals.clear()  # Clear the list after canceling
    print("All actions cleared.")
def main():
    try:
        while True:
            distance = get_proximity()
            print(f"DISTANCE: {distance}")
            if distance > 12:
                move_forward()
            elif distance <= 12:  # Object detected in range
                clear_all_actions()
                stop_robot()
                obj, confidence = object_recognition(4)  # Increased sample size for accuracy
                print(f"OBJECT DETECTED: {obj}")
                while get_proximity() >= 10:
                    move_forward()
                    time.sleep(0.1)  # Prevents excessive polling
                if obj != POKEBALL:
                    clear_all_actions()
                    rotate_right()
                    time.sleep(2)
                else:
                    clear_all_actions()
                    rotate_left()
                    time.sleep(2)
            else:
                clear_all_actions()
                stop_robot()  # Stops movement if too close to an obstacle
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nRobot Stopping...")
        clear_all_actions()
        stop_robot()
        node.destroy_node()
        rclpy.shutdown()
main()
