import cv2
import mediapipe as mp
import numpy as np
import serial
import time



# Define the serial port and baud rate
serial_port = "/dev/cu.usbmodem101"  # Replace with the correct port for your Arduino
baud_rate = 9600

# Open a serial connection to the Arduino   
arduino = serial.Serial(serial_port, baud_rate, timeout=1)
# serial_port1 = "COM3"  # Replace with the correct port for your Arduino
# baud_rate = 9600
        
# # Open a serial connection to the Arduino
# servo1_arduino = serial.Serial(serial_port1, baud_rate, timeout=1)
# Wait for the Arduino to initialize
time.sleep(2)

 # Close the serial connection on Ctrl+C
def normalize(value, new_min, new_max):
    x_min = 0  # Minimum value in the original range (assuming your input value is already in the original range)
    x_max = 500  # Maximum value in the original range (assuming your input value is already in the original range)
    
    normalized_value = (value - x_min) * (new_max - new_min) / (x_max - x_min) + new_min
    return normalized_value

# Example usage:


def calculate_bounding_box(landmarks, width, height):
    x_coords = [landmarks[i].x for i in range(len(landmarks))]
    y_coords = [landmarks[i].y for i in range(len(landmarks))]

    min_x = int(min(x_coords) * width)
    max_x = int(max(x_coords) * width)
    min_y = int(min(y_coords) * height)
    max_y = int(max(y_coords) * height)

    return min_x, min_y, max_x, max_y

def calculate_chest_center(landmarks, width, height):
    left_shoulder = landmarks[11]
    
    chest_center_x = int(left_shoulder.x * width)
    chest_center_y = int(left_shoulder.y * height)
    
    return chest_center_x, chest_center_y

cap = cv2.VideoCapture(0)
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    last_update_time = time.time()
    update_interval = 1  # Set the update interval in seconds

    while cap.isOpened():
        ret, frame = cap.read()
        frame = cv2.resize(frame, (500, 400))
        height, width, _ = frame.shape

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False

        results = pose.process(image)

        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        try:
            landmarks = results.pose_landmarks.landmark

            min_x, min_y, max_x, max_y = calculate_bounding_box(landmarks, width, height)
            chest_center_x, chest_center_y = calculate_chest_center(landmarks, width, height)

            cv2.rectangle(image, (min_x, min_y), (max_x, max_y), (0, 255, 0), 2)
            cv2.circle(image, (chest_center_x - int(110), chest_center_y), 5, (255, 0, 0), -1)

            current_time = time.time()
            if current_time - last_update_time >= update_interval:
                value_to_normalize = chest_center_x - int(110)  # Replace this with your value
                new_min = 55
                new_max = 180
                normalized_value = normalize(value_to_normalize, new_min, new_max)
                print("Normalized Value:", normalized_value)

                # Send the angle to the Arduino as bytes
                arduino.write(str(180 - int(normalized_value)).encode())
                
                last_update_time = current_time
                
                #command = input("Enter 'd' to control the second servo, or enter an angle for the first servo (0-180): ")
    # Send the mmand to the Arduino
                
            # user_input = input("Enter '1' to rotate the servo motor: ")    
                
            # Rotate the servo motor by sending a specific angle command to the Arduino
            # Replace 'YOUR_ANGLE_VALUE' with the desired angle value
            
            # print(f"Chest Center Coordinates: ({chest_center_x}, {chest_center_y})")

        except:
            pass
        
        cv2.imshow('Mediapipe Feed', image)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()

