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
The primary motivation to perform it (in the object detection context) is collision detection, given that the car will not hit anything at ground level, it is pointless to analyze ground data for searching objects, so it is removed to make the process more optimal. It is key to highlight that ground data is useful for other parts of the stack, such as occupancy mapping.

There are several techniques to perform ground filtering, being some of the most popular the following ones:

- **Curvature/normals-based**: The normal of a ground point is most likely to be pointing up (or down) directly, as proposed by Moosman et al. Computing the normals is expensive.
- **Model fitting**: A plane (with some tolerance) will be fitted in the scene using, for example, RANSAC. All the points considered inside the plane (withing a certain margin), will be considered ground. It is fast, but non-deterministic due to the random sampling. Moreover, the planar model may not be suitable for all kind of roads (i.e. urban ones because of the road drainages).
- **Ray-based**: Use rays/columns in the depth image, as done by Petrovskaya and Thrun, Bogoslavski, Tier IV or Cho et al. It is very fast, but may have accuracy issues because it only uses a small subset of the data and usually requires a high vertical resolution of the sensor. This is the one used in Autoware.Auto because it is fast and deterministic.
- **Other** approaches include factor graphs (very slow), voxels (throws away points), etc.

### [7.4.1. Ray-Based Ground Filtering](https://youtu.be/xSGCpb24dhI?t=1305)
The steps to perform it are the following (standard approach, by Petrovskaya and Thrun and Bogoslavskyi):

1. Either build a range image or break the point cloud into angular slices and call those *rays*.
2. Where the ground is local to the sensor will be defined.
3. The algorithm will scan through the points in a *ray* with increasing distance, having two posibilities:
   1. Having a quite flat triplet of points, so we are still *seeing* ground.
   2. Having a big change in the angle, so there is no more ground from this point in advance in the current *ray*.


The approach used by Tier IV and Cho et al. differs a bit, being their high-level steps these:

1. Build the range image or generate the point cloud slices as before.
2. Again, determine where the ground is local to the sensor.
3. Finally, scan through points in a ray with increasing distance aswell, but this time cones will be used:
   1. If the current point is close to the last one:
      1. Check if the point is in the local cone of the last point, in which case it will be considered of the same category as the last one (ground/non-ground). If it is not, it will be non-ground
   2. If they are not close, check the gloabl cone to determine if the point can be considered ground. This allows the algorithm to go from non-ground points to ground ones.

Autoware.Auto and Autoware.AI uses this *improved* ray-based ground filtering, including three additional improvements:

- **Thresholding Statistics**: The cones will grow infinitelly, so points further away from the vehicle were more likely to be considered ground, introducing more failures at long distances in the original algorithm. This was solved in Autoware by allowing the global cone to grow a certain ammount before transforming it into a cylinder. The local one is already bounded because if the new point is too far away, the global cone will be used.
- **Domain Knowledge (Heuristics)**: Autoware added 3 more labels, so instead of ground/non-ground, in their modified version a point can be `GROUND`, `PROVISIONAL_GROUND`, `NONGROUND`, `RETRO_NONGROUND` OR `NONLOCAL_NONGROUND`. Moreover, labels are updated based on new labels to make the algorithm way more expressive (while increasing its cost). The new rules are:
  - If a point is labeled with `RETRO_NONGROUND`, the label of the one before is switched to `NONGROUND`. This is done to require as many ground points as possible on distant objects. It will be assigned to a point when it is very vertical to the previous one, assuming that both points are part of a vertical structure.
  - If a point is labeled with `PROVISIONAL_GROUND`, it will become `NONGROUND` if the next point is `NONGROUND`. It improves the non-ground point to ground point transition. In particular, it helps when smaller objects are next to bigger ones, such as folliage in front of buildings. The label is assigned when a point is not considered ground, but still near to it (close to the last ground point or to the global cone).
  - If a point is labeled with `NONLOCAL_NONGROUND`, it will not cause previous `PROVISIONAL_GROUND` points to become `NONGROUND`. Improves the same aspect as the rule before.
- **Problem interpretation as a constraint satisfaction** (factor graph/markov network): Given that every point in a ray has a label, there will be some relationship between adjacent points and their labels so that a certain set of labels will be more likely. The labels of the point aswell as the distance between them will be used to compute the likelihood of them sharing a label. More info about reinterpreting algorithms as factor graphs can be found in the paper [MGM: A Significantly More Global Matching for Stereovision](https://www.researchgate.net/publication/282779014_MGM_A_Significantly_More_Global_Matching_for_Stereovision). `NOTE: This is considered but not yet implemented in Autoware.`.

The steps of Autoware's ground filtering algorithm are:

```python3
# Generate the rays from the input point cloud
rays = bin_into_rays(input_pointcloud)

# For each ray in the input point cloud 
for ray in rays:
  points = sort_points_by_radial_distance(ray)
  last_label = GROUND
  last_point = [0, 0]

  # For each point of the ray
  for point in points:
    # If the current point is close to the last one
    if points_are_close(point, last_point):
      # Copy last label unless the current point is very vertical with respect to the last one
      if points_have_steep_angle(point, last_point):
        point_label = RETRO_NONGROUND
      else: 
        point_label = last_label
    # If the current point is not local to the last one
    else:
      # If the current point is local to the last ground point
      if is_local(point, last_ground_point):
        point_label = PROVISIONAL_NONGROUND
      # If it is inside the global cone
      elif is_in_cone(point, global_cone):
        point_label = PROVISIONAL_GROUND
      # If it is not local to the last ground point and it is not in the global cone
      else:
        point_label = NONGROUND

  # Update last label
  if point_label == RETRO_NONGROUND:
    last_label = NONGROUND

  if last_label = PROVISIONAL_GROUND:
    if point_label == NONGROUND:
      last_label = NONGROUND
    else:
      last_label = GROUND
```


## [7.5. Clustering for Object Detection](https://youtu.be/xSGCpb24dhI?t=1995)

## 7.6. Shape Extraction

## 7.7. Using Detected Objects

## 7.8. Lab: The Autoware.Auto Object Detection Stack
