# Lecture 08: Camera-Based Object Perception
[![Autoware.Auto badge](https://img.shields.io/badge/Autoware-Auto-orange.svg)](https://www.autoware.auto)

This lecture is provided by [Michael REKE](https://www.linkedin.com/in/michael-reke-54a90589/), [Stefan SCHIFFER](https://www.linkedin.com/in/stefanschiffer/) and [Alexander FERREIN](https://dblp.uni-trier.de/pers/f/Ferrein:Alexander.html) and GjorgJi NIKOLOVSKI, from the [FH AAchen University](https://www.fh-aachen.de/en). The lecture is available in YouTube:

[![Lecture video](https://img.youtube.com/vi/OtjTa-meJ-E/0.jpg)](https://www.youtube.com/watch?v=OtjTa-meJ-E)

The lecture will cover how to use 2D camera data to detect objects in real scenarios. In particular, it will go through detecting lanes, vehicles and pedestrians with neural networks and the toolboxes that may be used for those purposes. Also, a lane detection node in ROS 2 will be created.

The provided slides in PDF and notes about the lab of the lecture can be found in the Apex.AI's [autowareclass2020 repository](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/08_Perception_Camera), in GitLab. The slides in PDF and the lab notes in MarkDown are included in the *resources* folder of this directory. The code used in the lecture video is available [here](https://gitlab.com/ApexAI/autowareclass2020/-/tree/master/code/src/08_Perception_Camera).


## 8.1. Camera Basics
### 8.1.1. Basic KPIs
- Resolution
- ...

### 8.1.2.Calculating Real World Points
- Monovision Systems
- Stereovision Systems
- Epipolar Coordinates


## 8.2. Camera Calibration
### 8.2.1. Installing the Camera System
### 8.2.2. Calibration Procedure - The Chessboard Pattern
### 8.2.3. Calculation of Intrinsic and Extrinsic Parameters


## 8.3. Neural Networks for Camera-Based Object Detection
### 8.3.1. Neural Networks Basics
### 8.3.2. Examples of DNNs
- YOLO
- ...
### 8.3.3. Available data sets
- KITTI
- Ford
- ...
### 8.3.4. Computation Problem
- Real-Time


## 8.4. Available Toolboxes
### 8.4.1. OpenCV: Basic Algorithm Toolbox
### 8.4.2. CUDA: GPU Deployment Toolbox
### 8.4.3. Higher Level Integrated Toolboxes
- NVIDIA AD Toolbox


## 8.5. Use Case: Lane Detection
### 8.5.1. Lane Detection Basics
### 8.5.2. Polynomial Lane-Fitting for Data Reduction
### 8.5.3. Step-by-Step Hands-on


## 8.6. Lab: Lane Detection with Real Data
- Readin Data from a Data Stream
- Calculate Real World Coordinates of the Lanes
- Polynomial Fitting of Detected Lanes
- Generating ROS 2 Messages
- Visualizatoin
  - RViz 2

