# Components Used 
- 2 x SG90 Servo Motors 
- 2 x CYS-S0250 Motors
- 4 x Mosfets (due to external power supply to motors)
- ESP-32 Microcontroller 
- Webcam Camera
# Working
- upload the code to your ESP32 and make the circuit according to the pin used in the code, keep the ESP32 connected to your PC (serial communication is used in this project) 
- Run the flask server
- Open localhost:5000 and there you go (you can now view the camera feed in the page and control the arm using your PC)
# Future Challenges
- Try using vision control and write an inverse kinematics function for the arm so that it is fully automated and vision controlled 
