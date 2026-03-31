# KFUPM-AutoWheel-CPS-IoT-QCar-competition

# [Youtube link for our Competition Submission – Stage 1]()

![Simulation Snapshot](https://github.com/Moussa-Rd/KFUPM-AutoWheel-CPS-IoT-QCar-competition/blob/main/KFUPM-AutoWheels.png)


---

# 📘 ReadMe

## 👋 Overview

Welcome to our **Stage 1 submission** for the **KFUPM-AutoWheel CPS-IoT-QCar-competition**.

We are a team from **King Fahd University of Petroleum and Minerals (KFUPM)**, and this project presents a complete autonomous driving simulation built on the **Quanser QCar platform** within a realistic urban environment. Our approach focuses on integrating solid engineering principles with **real-time control**, **computer vision**, and **system-level design**.

The solution is designed to meet all competition evaluation criteria while demonstrating a reliable and modular autonomous driving pipeline.

This repository brings together:
- **Intelligent perception** through color-based object detection  
- **Dynamic vehicle control** using tuned PD/PID controllers  
- A **custom-designed competition environment** implemented in Quanser QLabs  

The project showcases a cohesive system that combines perception, decision-making, and control for autonomous navigation in a simulated urban setting.

--- 

## 🧪 Testing Environment

| Component | Details |
|----------|---------|
| **Simulation** | Quanser Interactive Labs v2.24 (Oct 2025) |
| **Hardware** | MSI katana i7 14700HX, 16GB RAM, RTX 5060|
|**OS** | Windows 11 pro |
| **Detection Framework** |Advanced Image processing |
| **Programming Language** | Mathlab R2025b (Similink) |
| **Control** |  PD for Steering + PID for speed control |
| **Map** | Acc Setup_Real_Senario map |

---

## 🏁 Results

- ✅ **Accurate QCar path-following** with smooth ramped control  
- ✅ **Image processing methods detected** stop signs and traffic lights in varying conditions  
- ✅ **Real-time traffic light interaction** (red → stop, green → go)  
- ✅ **Stop signs triggered correct pauses**, then resumed autonomously  
- ✅ **System remained stable** throughout full trajectory duration


---

## 🧠 Development Process

### 1. Detection-based advanced Image processing:

We implement a **color-based object detection system in MATLAB Simulink** using **HSV thresholding** to detect traffic signs and traffic lights from an input RGB image.

The model first converts the RGB image into the **HSV color space**, which separates color information (**Hue**) from intensity-related components (**Saturation** and **Value**). This makes the detection process more reliable under different lighting conditions compared to direct RGB thresholding.

After conversion, the system applies **HSV thresholds** to isolate specific colors:
- **Red** for detecting stop signs and red traffic lights
- **Green** for detecting green traffic lights

The thresholding stage generates **binary masks** where the selected color regions are highlighted. These masks are then processed using **blob detection** to identify connected regions and extract useful properties such as:
- Area
- Bounding box
- Centroid

The detected blobs are further analyzed to classify objects in the scene. For example:
- **Red blobs with sign-like shape characteristics** are used for stop sign detection
- **Red and green blobs** are used to determine the current traffic light state

The model also includes display blocks for visualizing both the **original RGB image** and the **binary detection results**, which helps with debugging and threshold tuning.

Overall, this project demonstrates a simple and effective computer vision pipeline in Simulink by combining:
- **RGB-to-HSV conversion**
- **Color thresholding**
- **Blob analysis**
- **Basic object classification**

This makes it suitable for learning and experimenting with **real-time vision-based traffic scene analysis** in MATLAB/Simulink.

![ImageProcessing Snapshot](https://github.com/Moussa-Rd/KFUPM-AutoWheel-CPS-IoT-QCar-competition/blob/main/ImageProcessing.png)


---

![General Structure Snapshot](https://github.com/Moussa-Rd/KFUPM-AutoWheel-CPS-IoT-QCar-competition/blob/main/Struc.png)

---

### 2. Control System

The control system is composed of two main controllers responsible for vehicle motion: **Steering_Commander** and **Speed_Controller**.

- **Steering_Commander**  
  This block implements a **PD (Proportional-Derivative) controller** to correct the vehicle’s steering angle. It minimizes lateral deviation from the desired path and ensures smooth directional control. The derivative term helps reduce overshoot and improves responsiveness, especially during sharp turns.

- **Speed_Controller**  
  This block uses a **PID (Proportional-Integral-Derivative) controller** to regulate the vehicle’s speed. The controller parameters were tuned to achieve stable and responsive behavior in the virtual environment, balancing acceleration, steady-state error, and damping.

The output of the **Speed_Controller** is passed to a MATLAB function:

- **`taxiMissionThrottleGate`**  
  This function integrates control logic with mission-level objectives. It takes:
  - Throttle command (from the PID controller)
  - TaxiHub, Pickup, and DropOff coordinates  

  Based on the vehicle’s position relative to these waypoints, it dynamically adjusts the throttle to:
  - Slow down near pickup/drop-off points  
  - Stop when required  
  - Resume motion after completing actions  

Additionally, the control system incorporates inputs from the **image processing module**:

- Flags such as **traffic light state (red/green)** and **stop sign detection** are used in higher-level decision logic.
- These signals override or modify throttle commands to ensure safe behavior, such as:
  - Stopping at red lights  
  - Yielding when a stop sign is detected  

Overall, the control system combines **low-level control (PD/PID)** with **high-level decision logic**, enabling the vehicle to follow paths, regulate speed, and respond intelligently to traffic conditions.



![SpeedControl Snapshot](https://github.com/Moussa-Rd/KFUPM-AutoWheel-CPS-IoT-QCar-competition/blob/main/SpeedControl.jpg)

---


## 📂 Repository Structure

```
KFUPMAutoDrive-Simulation/

├── VIRTUAL_self_driving stack_V2.slx/    # For running the navigation 
│── example_sdcs_path.m/                  # For the A* planner  
├── Setup_Qcar2_Params.m                  # To upload the needed parameters for the SimLink file into the workspace
├── Setup_Real_Senario.m                  # For uploading the map 
└── README.md
```

---

### 📋 Prerequisites

Before running the project, make sure you have:

- MATLAB & Simulink installed  
- Quanser **QLabs** installed and working  
- Required toolboxes (Simulink, Image Processing, etc.)  
- All project files in the same working directory  

---

### 🚀 Step-by-Step Execution

#### 1. Start QLabs
- Open **Quanser QLabs**
- Ensure:
  - The environment is loaded properly  
  - No previous real-time models are running  
  - The simulation is idle and ready  

---

#### 2. Generate the Navigation Path
- Run the following script 'example_sdcs_path.m'
- Enter the pickup node
- Enter the drop-off node
- Run: 'Setup_Qcar2_Params.m'
- After that, initialize the Simulation Scenario by running 'Setup_Real_Senario.m'
- Finally run the Main Simulink Model 'VIRTUAL_self_driving_stack_V2.slx'




**Ps:** For any further details, check the YouTube link at the top
