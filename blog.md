
# 14/10/2025 - Redesign

The entire arm has been redesigned for a simpler layout and to allow easy mounting of ArUco markers. 

<img width="774" height="439" alt="image" src="https://github.com/user-attachments/assets/00aa133d-d4f2-44c9-b089-2b43fa7129a8" />

<img width="846" height="572" alt="image" src="https://github.com/user-attachments/assets/f8b257f6-c469-4ef1-b9c0-d137fea4d07f" />

<img width="3000" height="4000" alt="1" src="https://github.com/user-attachments/assets/ef45c316-0210-4ae9-9fc4-dbc024927222" />

<img width="4000" height="3000" alt="2" src="https://github.com/user-attachments/assets/1ae99fb7-8e7a-4c8f-9788-28eff2105fec" />

<img width="4000" height="3000" alt="3" src="https://github.com/user-attachments/assets/7a064ffb-0223-4513-8f73-5e5b525dfecc" />

<img width="4000" height="3000" alt="4" src="https://github.com/user-attachments/assets/c85e424e-782f-460a-a753-d0a242e32697" />

# 20/09/2025 - YOLO card detection

The card detection is complete. 

- The dataset contains 828 images of the four cards, with differing orientation, position, lighting and card count. There are also negative images to prevent false positives during deployment of the model. 
- The images were labelled using [Label Studio](https://labelstud.io/), shown below:
<img width="1262" height="709" alt="image" src="https://github.com/user-attachments/assets/60b17762-b34d-4c0e-9a69-270be1b8efab" />

- The images were randomly split into 80% training and 20% validation. 
- I chose to use YOLO11n to optimise frame rate for use on a raspberry pi; a comparison of the different YOLO11 sub-models is shown below:
<img width="950" height="490" alt="yolo11 performance" src="https://github.com/user-attachments/assets/d5176e7d-85c6-4ad5-ba1b-1632904a6b86" />

- I trained the model locally on an Nvidia RTX 3060Ti GPU, with 40 epochs, which is shown below:
<img width="2400" height="1200" alt="results" src="https://github.com/user-attachments/assets/dcbe1c3f-dfcf-41fa-8163-128bce8cd3d2" />

- I wrote a python program to detect the cards in real time, and also combined the Aruco marker detection; a demo is shown below:

https://github.com/user-attachments/assets/abf8410f-be76-4347-8a52-7dc9d73a1910

# 19/09/2025 - Suction pipeline

The suction pipeline contains:
- Arduino uno with a sensor shield
- Solenoid valve
- DC vacuum pump
- Suction cup
- Silicone tubing

The pump and valve are controlled using the Servo arduino library. The suction cup will be used as the end effector for the robotic arm, to pick up the playing cards.

![suction pipeline](https://github.com/user-attachments/assets/aaa81f64-aaaf-4d7f-bcf5-3732c60bca5c)

# 26/08/2025 - Raspberry Pi & Arduino Communication

I set up communication between the raspberry pi & arduino, using a serial connection.
I also incorporated the inverse kinematics script, so that the joint angles are calculated on the pi, then sent to the arduino.

<img width="1026" height="416" alt="image" src="https://github.com/user-attachments/assets/c963547b-26d3-4511-ba01-3adb8c6a56d0" />

# 03/08/2025 - Image dataset

In order to train a YOLO model to detect playing cards (and which suit it is), I must gather many example images:
- The arm will sort playing cards by suit, so use the 10s because they have the most identifiable objects (e.g. hearts).
- Must be in the same conditions as the task will be carried out in to maximise performance, same lighting, same background, etc
- I aim to gather around 400 images, since the arm will be working in a controlled environment - the background will not change during detection
  
Then the next steps are:
- The images must be labelled, I'm using [Label Studio](https://labelstud.io/)
 to achieve this
- Images must be split into training and validation sets
- Then I will train the model using an anaconda virtual environment, using a local GPU

<img width="769" height="1025" alt="image" src="https://github.com/user-attachments/assets/668da406-289a-4850-8649-660c76f9faa1" />

# 02/08/2025 - Base completion

The base has been completed apart from the lid. The motors are mounted outside the turret, as an improvement on the previous version of the arm. It allows any NEMA 17 motor to be mounted without changing the design, and also allows easier access to the cables.

It features:
- Rotating turret to accommodate a counterweight
- Centering mechanism
- Timing belt driven rotation
- Belt tensioner
- NEMA 17 motors
- Limit switch for motor homing

<img width="1063" height="530" alt="image" src="https://github.com/user-attachments/assets/96c9ba07-a65f-419f-bced-29ea7c7a1b2d" />

<img width="565" height="577" alt="image" src="https://github.com/user-attachments/assets/d70effa1-7c7d-4b0e-a380-0fc4a3f99f20" />

<img width="781" height="649" alt="image" src="https://github.com/user-attachments/assets/364ff8c7-144b-454f-85e7-bff3df2ccb04" />
