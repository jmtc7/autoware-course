# Lecture 07: LiDAR-Based Object Detection
[![Autoware.Auto badge](https://img.shields.io/badge/Autoware-Auto-orange.svg)](https://www.autoware.auto)

This lecture is provided by [Christopher HO](https://www.linkedin.com/in/christopher-ho-0608683a/) and [Gowtham RANGANATHAN](https://www.linkedin.com/in/gowtham-ranganathan-37a60782/), Software Engineers at Apex.AI. The lecture is available in YouTube:

[![Lecture video](https://img.youtube.com/vi/xSGCpb24dhI/0.jpg)](https://www.youtube.com/watch?v=xSGCpb24dhI&list=PLL57Sz4fhxLpCXgN0lvCF7aHAlRA5FoFr&index=8)

The lecture will provide an overview of how to detect objects for Autonomous Vehicles (AVs) using a LiDAR sensor. In particular the covered topics will be the purpose of object detection, how does it fit on AVs and how to use it with Autoware.Auto. The Autoware.Auto object detection stack will be explained, including how it works, how to use and how to tune it.

The provided PDFs can be found in the Apex.AI's [autowareclass2020 repository](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/07_Object_Perception_LIDAR), in GitLab. The PDFs are included in the *resources* folder of this directory. There is oen PDF for each lecture section (8 in total).


## [7.1. Object Detection and the Autonomous Driving Stack](https://youtu.be/xSGCpb24dhI?t=30)
In general, an Autonomous Vehicle (AV) should drive safely towards a goal, which means not to hit things and to perform safe behaviors keeping in mind the other participants in its environment. Object detection is useful to detect things that the car must not hit and to detect other participants to be able to analize these detections and predict behaviors and plan the one of the ego-car in other layers of the stack. In relation with it, an Autonomous Driving (AD) Stack is, in general, composed by four tasks: Sense (perceive the environment), understand (generate a world model), plan (decide how to behave in the world model) and actuate (interact with the world).

Going into a bit more of detail, the **perception** layer is composed by sensor drivers and data preprocessing (filtering, cleaning, formatting, etc.), while the **understanding** phase includes object detection and tracking and ego car localization and motion estimation. 

The inputs of the object detection is most of the time 3D data obtained from sensors such as LiDARs, RaDARs, stereo and depth cameras, Structure from Motion (SfM) with scale estimation, Deep Learning (DL)-processed images or even combinations between these.


## [7.2. A Classical LiDAR Object Detection Stack](https://youtu.be/xSGCpb24dhI?t=380)
Most Computer Vision problems have two main ways of being solved nowadays, the Deep Learning (DL) based approaches and the classical ones. **DL** offers a very interesting way of achieving cutting edge performance, while it requires a lot of (usually) labeled data and depends in the availability of a GPU to run. **Classical** approaches are easier to understand and improve, do not require data (but for testing), but they usually have a lot of parameters to be tuned. Another big difference is the vulnerability of DL-approaches towards adversarial attacks, as shortly explained [here](https://medium.com/syncedreview/researchers-fool-lidar-with-3d-printed-adversarial-objects-3f92962f0f96A) and more deeply analized [here](https://arxiv.org/pdf/1907.06826.pdf) and [here](https://deepai.org/publication/physically-realizable-adversarial-examples-for-lidar-object-detection).

Autoware implements classical approaches doe to its simpliness, its absence of big dependencies (such as TensorFlow or PyTorch and this adversarial attacks. In this classical approach, the LiDAR-obtained data is forwarded to the drivers before being transformed and filtered to be finally fused in a common environment representation. All this happens in the *sensing* layer. Next, in the *undestanding* one, the steps are to filter the ground (which usually only contributes as noise), detect objects (by clustering) and extract their shapes (to simplify the representation of the information).

## [7.3. Preprocessing LiDAR Data](https://youtu.be/xSGCpb24dhI?t=715)
The data preprocessing is located in the *sensing* layer of the stack, being composed of the transform, filter and fusion nodes in Autoware. The target of this step is to forward the minimum ammount of information that allow the system to produce the correct results, which is achieved by:

- Removing useless data.
- Removing problematic/bad data.
- Removing redundant data.
- Producing a single and consistent input for the next layer.

Some common preprocessing operations for LiDAR-gathered data are:

- **Range-Based Filtering**: It consists in removing the points that are too close or too far from the sensor. The ones that are too close can be from the ego vehicle and the ones too far away will potentially have no context. This includes the parameters *r_min* and *r_max*, being the minimum and maximum distances at which a detection may occur to be considered as valid data.
- **Angle-Based Filtering**: Remove the points outside a certain angular region, defined by the parameters *theta_min* and *theta_max*. It mitigates the *flyging birds* effect (amongst others), which consists in false positives in a LiDAR caused by the beams of another LiDAR.
- **Downsampling**: Combine several measurements if they are too close to each other to avoid data redundancy. It can be done by voxel grid approaches (the most common) or by random sampling ones (not recommended because of their randomness and the bias they introduce).
- **Point Cloud Fusion**: When using several LiDAR sensors, it is useful to combine the readings of each of them into a single bigger one. When driving at higher speeds, it is key to take into account when each measure was taken and what the ego motion was between these timestamps to properly align the measurements. At very high speed (such as in race cars), the time stamp of each laser beam might be considered.


## [7.4. Ground Filtering](https://youtu.be/xSGCpb24dhI?t=1075)

## 7.5. Clustering for Object Detection

## 7.6. Shape Extraction

## 7.7. Using Detected Objects

## 7.8. Lab: The Autoware.Auto Object Detection Stack
