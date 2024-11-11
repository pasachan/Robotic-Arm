from flask import Flask, render_template, request, Response
import serial
import time
import cv2
import math

# Set up serial communication with ESP32
ser = None
try:
    ser = serial.Serial('COM10', 115200, timeout=1)  # Open the serial port for communication
    time.sleep(2)  # Wait for serial connection to initialize
except serial.SerialException as e:
    print(f"Error: {e}")
    ser = None

# Initialize the Flask app
app = Flask(__name__)

# Current angles for each servo motor
angles = {
    "link2": 0,
    "gripper": 0,
    "link1": 0,
    "base": 0,
}


# Initialize the camera
camera = cv2.VideoCapture(1)  # Use the first camera (change the index if needed)

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
                    x_center_full = x_center + x_start
                    y_center_full = y_center + y_start

                    # Calculate joint angles
                    print(f"Center Coordinates (Full FOV): ({x_center_full}, {y_center_full})")

    return img, objectInfo

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/control')
def control():
    part = request.args.get('part')
    direction = request.args.get('direction')

    # Update angle based on direction
    if direction == 'forward':
        angles[part] = min(angles[part] + 5, 180)  # 10-degree movement forward
    elif direction == 'backward':
        angles[part] = max(angles[part] - 5, 0)  # 10-degree movement backward

    # Send the command to the ESP32 only if the serial port is open
    if ser and ser.is_open:
        command = f"{part}:{angles[part]}\n"
        ser.write(command.encode())
        return f"Sent {command}"
    else:
        return "Error: Serial port not open"

@app.route('/reset')
def reset():
    # Set all angles to 0
    for part in angles:
        angles[part] = 0
        command = f"{part}:0\n"
        if ser and ser.is_open:
            ser.write(command.encode())
    return "Reset all angles to 0"
@app.route('/performPick')
def performPick():
    if ser and ser.is_open:
        ser.write("gripper:180\n".encode())
        time.sleep(2)
        ser.write("base:150\n".encode())
        time.sleep(2)
        ser.write("link1:40\n".encode())
        time.sleep(2)
        ser.write("link2:160\n".encode())
        time.sleep(2)
        ser.write("gripper:15\n".encode())
    return "Pickup done"
@app.route('/performPlace')
def performPlace():
    if ser and ser.is_open:
        ser.write("link1:60\n".encode())
        time.sleep(2)
        ser.write("base:10\n".encode())
        time.sleep(2)
        ser.write("link1:40\n".encode())
        time.sleep(2)
        ser.write("gripper:100\n".encode())
    return "Place done"
@app.route('/performPick2')
def performPick2():
    if ser and ser.is_open:
        ser.write("base:10\n".encode())
        time.sleep(2)
        ser.write("gripper:100\n".encode())
        time.sleep(2)
        ser.write("link1:45\n".encode())
        time.sleep(2)
        ser.write("link2:160\n".encode())
        time.sleep(2)
        ser.write("gripper:15\n".encode())
    return "Pickup done"
@app.route('/performPlace2')
def performPlace2():
    if ser and ser.is_open:
        ser.write("link1:55\n".encode())
        time.sleep(2)
        ser.write("base:150\n".encode())
        time.sleep(2)
        ser.write("link1:35\n".encode())
        time.sleep(2)
        ser.write("gripper:100\n".encode())
    return "Place done"
# Route for video feed
def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            height, width, _ = frame.shape

            # Calculate cropping coordinates to center the FOV area
            x_start = (width - FOV_WIDTH) // 2
            y_start = (height - FOV_HEIGHT) // 2
            x_end = x_start + FOV_WIDTH
            y_end = y_start + FOV_HEIGHT

            # Draw FOV boundary rectangle on the full frame
            cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), (255, 0, 0), 2)

            # Annotate the coordinates of the rectangle's corners
            cv2.putText(frame, f"({x_start}, {y_start})", (x_start, y_start - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            cv2.putText(frame, f"({x_end}, {y_end})", (x_end - 100, y_end + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            # Crop the image to the custom FOV area
            frame_cropped = frame[y_start:y_end, x_start:x_end]

            # Perform object detection within the cropped area
            result, objectInfo = getObjects(frame_cropped, 0.45, 0.2, x_start, y_start)

            # Encode the frame in JPEG format
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()

            # Use multipart/x-mixed-replace to stream the video feed
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(debug=False)
