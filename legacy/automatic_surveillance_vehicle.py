"""Formatted snapshot of the original proof-of-concept implementation.

The maintained implementation is under ``src/surveillance_vehicle``. This file
is retained only to preserve the project's development history.
"""

import time

import cv2
import mediapipe as mp
import serial

SERIAL_PORT = "/dev/cu.usbmodem101"
BAUD_RATE = 9600


def normalize(value, new_min, new_max):
    x_min = 0
    x_max = 500
    return (value - x_min) * (new_max - new_min) / (x_max - x_min) + new_min


def calculate_bounding_box(landmarks, width, height):
    x_coords = [landmark.x for landmark in landmarks]
    y_coords = [landmark.y for landmark in landmarks]
    return (
        int(min(x_coords) * width),
        int(min(y_coords) * height),
        int(max(x_coords) * width),
        int(max(y_coords) * height),
    )


def calculate_chest_center(landmarks, width, height):
    left_shoulder = landmarks[11]
    return int(left_shoulder.x * width), int(left_shoulder.y * height)


def main():
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    capture = cv2.VideoCapture(0)
    pose_module = mp.solutions.pose

    with pose_module.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        last_update_time = time.time()
        while capture.isOpened():
            ok, frame = capture.read()
            if not ok:
                break
            frame = cv2.resize(frame, (500, 400))
            height, width, _ = frame.shape
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
            results = pose.process(image)
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            if results.pose_landmarks:
                landmarks = results.pose_landmarks.landmark
                min_x, min_y, max_x, max_y = calculate_bounding_box(landmarks, width, height)
                chest_x, chest_y = calculate_chest_center(landmarks, width, height)
                cv2.rectangle(image, (min_x, min_y), (max_x, max_y), (0, 255, 0), 2)
                cv2.circle(image, (chest_x - 110, chest_y), 5, (255, 0, 0), -1)

                if time.time() - last_update_time >= 1:
                    angle = normalize(chest_x - 110, 55, 180)
                    arduino.write(str(180 - int(angle)).encode())
                    last_update_time = time.time()

            cv2.imshow("Mediapipe Feed", image)
            if cv2.waitKey(10) & 0xFF == ord("q"):
                break

    capture.release()
    arduino.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
