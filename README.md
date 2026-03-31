# KFUPM-AutoWheel-CPS-IoT-QCar-competition

# [Youtube link for our Competition Submission – Stage 1]()

![Simulation Snapshot](https://github.com/Moussa-Rd/KFUPM-AutoWheel-CPS-IoT-QCar-competition/blob/main/KFUPM-AutoWheels.png)


---

# 📘 ReadMe

## 👋 Overview

Welcome to our Stage 1 submission for the **KFUPM Self-Driving Challenge**!

As students from **King Fahd University of Petroleum and Minerals (KFUPM)**, we’ve built a complete autonomous vehicle simulation using the **QCar platform** in a realistic urban environment. Our focus was to combine strong engineering principles with real-time control, computer vision, and system integration. This solution meets all the evaluation criteria.

This repository combines **intelligent object detection**, **dynamic vehicle control**, and a **custom competition environment** in Quanser's QLabs.

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

### 1. Object Detection
- 

### 2. Control System
- We have 2 main controllers, the first one 'Stearing_Commander.' It uses a PD controller and corrects the steering command, that weill be later used for the speed controller. Secondly, 'Speed_Controller', which is a PID controller. The parameters were tuned to get the best possible performance for the virtual stage.
- The output of 'Speed_Controller' will be used in a MATLAB function named **taxiMissionThrottleGate**  along with the TaxiHub, Pickup, and DropOff coordinates to handle the PickUp/DropOff scenario.
- The flags obtained from the image processing module were also used in the logic of the Trafficlight/Stop Logic.

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

## 🛠️ How to Run
 
1. **Start QLabs**

  - Make sure QLabs is open, and all previous real-time models are terminated.
   
2.**Launch experiment**
  - First run example_sdcs_path.m by specifying the pickup node and dropoff node, this will return the full path saved in a 'sdcs_taxi_mission.mat' file.
  - Running Setup_Qcar2_Params.m to upload the different parameters needed for the navigation blocks, including the generated path from example_sdcs_path.m
  - After that,  run Setup_Real_Senario.m
  - Finally running the main file VIRTUAL_self_driving stack_V2.slx
    
---

**Ps:** For any further details, check the YouTube link At the top
