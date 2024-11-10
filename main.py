import cv2
import math
import numpy as np
import serial
import time

# Set up serial communication with ESP32
esp32_serial = serial.Serial('COM10', 115200, timeout=1)  # Replace 'COM3' with your ESP32's port
time.sleep(2)  # Wait for serial connection to initialize

# Load class names for object detection
classNames = []
classFile = "./coco.names"
with open(classFile, "rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

# Paths for configuration and weights files
configPath = "./ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "./frozen_inference_graph.pb"

# Load the network
net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

# Define FOV boundaries (e.g., center area of 320x240 pixels)
FOV_WIDTH = 320
FOV_HEIGHT = 240 

import math

def calculate_joint_angles(x,y):
    # Given parameters
    Px = x-40
    Py = y+90
    Pz = -210
    a2 = 138 * 2
    a3 = 235*2

    # Calculate C3 and S3
    C3 = (Px**2 + Py**2 + Pz**2 - a2**2 - a3**2) / (2 * a2 * a3)
    theta3 = math.degrees(math.acos(C3))
    # Calculate theta1
    theta1 = math.degrees(math.atan((x-40)/(y-90)))
    theta2 = math.atan(Pz/(Px**2 + Py**2)) - math.atan((a3*math.sin(theta3))/(a2+a3*math.cos(theta3)))
    # Display the results
    print(f"theta1 = {theta1:.2f} degrees")
    print(f"theta2 = {theta2:.2f} degrees")
    print(f"theta3 = {theta3:.2f} degrees")

    return theta1, theta2, theta3


def getObjects(img, thres, nms, x_start, y_start, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nms)
    if len(objects) == 0: 
        objects = classNames
    objectInfo = []
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            className = classNames[classId - 1]
            if className in objects:
                x, y, w, h = box
                
                # Ensure the bounding box stays within the cropped area
                if x < 0: x = 0
                if y < 0: y = 0
                if x + w > FOV_WIDTH: w = FOV_WIDTH - x
                if y + h > FOV_HEIGHT: h = FOV_HEIGHT - y
                
                # Calculate object area and check if it's too large
                object_area = w * h
                if object_area > (0.8 * FOV_WIDTH * FOV_HEIGHT):  # Ignore if the object is larger than 80% of FOV
                    continue

                objectInfo.append([box, className])
                if draw:
                    # Draw the bounding box within the cropped area
                    cv2.rectangle(img, (x, y), (x + w, y + h), color=(0, 255, 0), thickness=2)
                    cv2.putText(img, "Object Detected", (x + 10, y + 30),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(img, str(round(confidence * 100, 2)), (x + 200, y + 30),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                    
                    # Calculate the center of the bounding box
                    x_center = (x + w) // 2
                    y_center = (y + h) // 2
                    

                    
                    # Invert the y coordinate (since the origin is at the top-left of the image)
                   
                    
                    # Coordinates of the bounding box corners (relative to the full FOV)
                    top_left = (x + x_start, y + y_start)
                    top_right = (x + w + x_start, y + y_start)
                    bottom_left = (x + x_start, y + h + y_start)
                    bottom_right = (x + w + x_start, y + h + y_start)
                    
                    # Invert the y coordinates for the corners (since the origin is at the top-left)
                    top_left = (top_left[0], -top_left[1])
                    top_right = (top_right[0], -top_right[1])
                    bottom_left = (bottom_left[0], -bottom_left[1])
                    bottom_right = (bottom_right[0], -bottom_right[1])
                    x_center_full = (top_left[0]+top_right[0])/2
                    y_center_full =  (top_left[1]+bottom_left[1])/2
                    theta1, theta2, theta3 = calculate_joint_angles(x_center_full, y_center_full)
                    # Print bounding box corner coordinates and center relative to full FOV
                    print(f"Center Coordinates (Full FOV): ({x_center_full}, {y_center_full})")
                    print(f"theta1, theta2, theta3: ({theta1}, {theta2}, {theta3})")

    return img, objectInfo

def main():
    # Initialize video capture
    cap = cv2.VideoCapture(0)  # Change to 1 if using external camera
    cap.set(3, 640)  # Set camera width
    cap.set(4, 480)  # Set camera height

    while True:
        success, img = cap.read()
        if not success:
            break

        height, width, _ = img.shape

        # Calculate cropping coordinates to center the FOV area
        x_start = (width - FOV_WIDTH) // 2
        y_start = (height - FOV_HEIGHT) // 2
        x_end = x_start + FOV_WIDTH
        y_end = y_start + FOV_HEIGHT

        # Draw FOV boundary rectangle on the full frame
        cv2.rectangle(img, (x_start, y_start), (x_end, y_end), (255, 0, 0), 2)
        
        # Annotate the coordinates of the rectangle's corners
        cv2.putText(img, f"({x_start}, {y_start})", (x_start, y_start - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv2.putText(img, f"({x_end}, {y_end})", (x_end - 100, y_end + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # Crop the image to the custom FOV area
        img_cropped = img[y_start:y_end, x_start:x_end]

        # Perform object detection within the cropped area
        result, objectInfo = getObjects(img_cropped, 0.45, 0.2, x_start, y_start)

        # Send signal to ESP32 if an object is detected
        if objectInfo:
            print("Object detected!")
            esp32_serial.write(b'1')  # Send '1' to turn on the LED on ESP32
        else:
            esp32_serial.write(b'0')  # Send '0' to turn off the LED on ESP32

        # Display the frame with detected objects and boundary limits
        cv2.imshow("Output", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    esp32_serial.close()

if __name__ == '__main__':
    main()
