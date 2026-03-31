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

### 🔍 Perception Layer
- **For Image processing **, we've done the following:
  - 
    

---

## 🧠 Development Process

### 1. Object Detection
- I

### 2. Control System
- 

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
     Make sure QLabs is open, and all previous real-time models are terminated.
2.**you**
     
3.**Launch experiment**
  - First run Setup_Real_Senario.m
  - Then run A* planner to generate the full path, starting from fixing the starting node (TaxiHub) to predefined pickup and dropoff points 
  - Running Setup_Qcar2_Params.m to upload the different parameters needed for the navigation blocks, including the generated path from example_sdcs_path.m
  - Finally running the main file VIRTUAL_self_driving stack_V2.slx 


---

## 🗂️ File Descriptions
---

| File                                    | Description                                                                                                                                                                                                                                                                                                        |
|-----------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `VIRTUAL_self_driving stack_V2.slx`     |         |
| `example_sdcs_path.m`                   |  |
| 'Setup_Qcar2_Params.m '                 ||
|'Setup_Real_Senario.m'                   ||
