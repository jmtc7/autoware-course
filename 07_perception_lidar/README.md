# Lecture 07: LiDAR-Based Object Detection
[![Autoware.Auto badge](https://img.shields.io/badge/Autoware-Auto-orange.svg)](https://www.autoware.auto)

This lecture is provided by [Christopher HO](https://www.linkedin.com/in/christopher-ho-0608683a) and [Gowtham RANGANATHAN](https://www.linkedin.com/in/gowtham-ranganathan-37a60782), Principal and Senior Software Engineers at Apex.AI, respectively. The lecture is available in YouTube:

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
This step is needed to segmentate the input nonground point cloud into different objects to be able to evaluate possible collisions, predict individual behaviors, work with more manegable ammounts of data, etc. The clustering is done by grouping points together based in some metric.

There are not many clustering techniques for point clouds. Some of those approaches are:
- **Voxel-based** region growing technique.
- **Image space-based** region growing technique based on the angle metric.
- **Euclidean Clustering**, a region growing technique in the Euclidean Space based on the euclidean distance metric. This is the one used in Autoware.Auto and these are its steps:
   1. Creating a KD-Tree represeentation for the input point cloud `P`.
   2. Set up an empty list of clusters `C` and a queue `Q` of the points that need to be checked.
   3. For every point `p_i` in the input `P`, perform:
      1. Add `p_i` to the queue `Q`.
      2. For every point `p_i` in `Q`, do:
         1. Search for the set `P_k` of neighbors of `p_i` in a sphere with radius `r < d_th`.
         2. For every neighbor `p_ki` in `P_k`, check if it has already been processed and, if not, add it to `Q`.
      3. When the list of all points in `Q` has been processed, add `Q` to the list `C` and reset `Q`.
   4. The algorithm will end when all the points `p_i` in `P` have been processed, being part of one of the clusters in `C`.

Useful references are the paper [Semantic 3D Object Maps for Everyday Manipulation in Human Living Environments](https://www.researchgate.net/publication/36420830_Semantic_3D_Object_Maps_for_Everyday_Manipulation_in_Human_Living_Environments) and the [Point Cloud Library (PCL)](https://pointclouds.org).

To sum it up, the Euclidean Clustering algorithm starts with an empty cluster, adds a point to it, finds the points *near* this point and adds them to the cluster, searching neighbours of all the points in the cluster. When there are no more points to be processed in the cluster, it will be accepted or rejected depending on how many points it contains. This will be repeated for another point outside the cluster until every point in the input cloud belongs to a cluster. It can be understood as a graph where each point is a vertex and each cluster will be a connected sub-graph. Two vertices will be connected if they are *near* to one another.

Given that the algorithm searches for all the neighbors within a certain distance of the current point (near neighbors), not just the nearest one, the optimal data structure is an **integer lattice** or **spatial hash** (instead of a KD-Tree, which would be optimal for a nearest neighbor search). This structure consists in dividing the space into voxels, finding the voxel where the current point is in, find the adjacent voxels to it and iterate over all the points contained in the current or adjacent voxels, which will be included in the cluster. Using this structure instead of a KD-Tree makes it possible to reduce the algorithm complexity from *K * log(n)* to *K * n*, where *n* is the number of points and *K* the number of neighbours. Going from a linearithmic to a linear complexity is a big deal because the point clouds used in AVs usually contains a lot of points (*n* ~= [10k, 1M]).

As additional notes, Autoware.AI performs this in over 100 ms with aggressive downsampling and Autoware.Auto achieves 10 ms without downsampling.


## [7.6. Shape Extraction](https://youtu.be/xSGCpb24dhI?t=2570)
It is necessary because it is hard to work, transform and communicate using the raw clusters/blobs extracted from the last step. Moreover, their size is big and unbounded. 

There are several ways of extracting a form, such as trying to bound a pointcloud in a circle, axis-aligned bounding box (BB), oriented BB or a convex hull. The more to the right of the list, the better they represent the object but the harder to compute and to perform collision/overlap checking. This is why oriented BBs are usually the chosen option. There are several ways of **computing oriented BBs**:

- **Rotating Calipers**: Compute the 2D convex hull of the object to generate a 2D BB with minimum perimeter or area. Since it only considers boundary points, it is very sensitive to noise. Moreover, there are shapes that will lead to bad results, such as if two sides of a car are seen (L-shaped points), the resulting BB will be in a diagonal of the car, including a lot of empty space and leaving half of the car outside of the box.
- **Principal Component Analysis (PCA)**: Use PCA to compute the major and minor axes of the BB. It will be more robust to noise but will still fail in cases such as the L-shaped sets of points.
- **Optimization Approaches**: The approach of Shen et al. searches for a box and a partition that best fits the L-shape. However it relies on a single scanning LiDAR (so it will not work well when using more than one LiDAR). Zhang et al. relaxes this assumption, but introduces discretization error.

Autoware.Auto uses the approach of Shen et al. with a few pre-processing steps. First, the principal component of the blob is computed, then the points are sorted along this axis, pretending all the points came from a single scan of a LiDAR, so Shen's algorithm can be used as normal. This makes the algorithm more expensive, but it avoids the discretization error and works on every poipnt cloud.

As additional notes, other approaches for shape extraction are:

- **Convex Hulls**: It is only loosely bounded in memory (being the upper bound the number of points in the point cloud, since every one of them could be from the edge of the object). This makes it quite inconvinient to move data. Autoware.Auto, however, implements the Monotone Chain algorithm, which is of this type.
- **Minimum Volume Bounding Ellipsoids**: It is similar to use oriented BBs, but they require to solve an optimization problem to perform collision checking. An example is the one proposed by Kumar and Yildirim. Its advantage is that ellipses fit better than BBs in optimization frameworks, avoiding things such as looking at second order cones.
- **Super Quadratics** (i.e. Pascoal et al): It allows representing complex shapes, such as concave ones, but it requires to solve a hard optimization problem.

Currently, the open problem is how to handle large concave objects, such as the inside of a small garage.


## [7.7. Using Detected Objects](https://youtu.be/xSGCpb24dhI?t=3215)
The biggest use cases of object detection are:

- **Classification**, which is helpful for predicting the behaviors of the other traffic participants. It can also influence the behavior planning of the system. The detected clusters/blobs can be classified combining the information with camera data or other classifiers.
- **Preprocessing**: The clusters can be used as Regions of Interest (RoIs), which can be used for classification, localization landmarks, marker/signals detection/classification, etc.
- **Tracking**: It consist in fusing instantaneous detections over time. The main types are:
  - Tracking-by-assignment, such as data associations method (hungarian algorithm, GNN, etc.) and state estimation (Kalman Filter, Particle Filter, etc.)
  - Combined tracking methods, such as the Multiple Hypothesis Tracking by Khan et al and Kim et al.
- **Collision detection**: The main usage of object detection. Algorithms implementing it are SAT and GJK (+EPA). There are several ways of using it, as exposed in motion planning literature, such as Mirtich (1997) or Lin and Gottschalk.


## [7.8. Lab: The Autoware.Auto Object Detection Stack](https://youtu.be/xSGCpb24dhI?t=3470)
[Gowtham RANGANATHAN](https://www.linkedin.com/in/gowtham-ranganathan-37a60782) will demonstrate how to use the perception stack using the LGSVL simulator. The simulator setup is explained in the [*Tutorials* section of the Autoware.Auto documentation](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/lgsvl.html), which can be accesed from the [*Documentation* tab of the Autoware.Auto website](https://www.autoware.auto). To run the simulator, the requirements are:

- ADE 4.2.0 or later
- NVidia graphics card
- Native GPU support installed if using Docker engine 19.03 or later
- nvidia-docker2 installed if using Docker engine below 19.03

In order to run the simulator, the following orders should be executed from inside the `~/adehome/AutowareAuto/` directory, which should be in our system after the [Lecture 01](https://github.com/jmtc7/autoware-course/tree/master/01_development_environment):

```bash
$ source .aderc-lgsvl  # Source the LGSVL-specific ADE rc file
$ ade start --enter  # Start and enter the ADE environment
ade$ RMW_IMPLEMENTATION=rmw_cyclonedds_cpp /opt/lgsvl/simulator  # Start the simulator
```

A window will appear, offering a button to open a browser. However, it does not work, so all the simulation configuration can be accessed by opening manually [http://127.0.0.1:8080](http://127.0.0.1:8080) in a browser. It is necessary to sign up if its the first time using it. Next, tabs to configure maps, vehicles, clusters and simulations will be available. The minimum required things are having one valid map and one valid vehicle. Instructions in how to import them can be found in the [LGSVL tutorial](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/lgsvl.html) of the Autoware.Auto documentation. The steps are:

- Click the *Add new* button in the *Simulations* tab.
- Enter a name for the simulation.
- Go to the *Map & Vehicles* tab.
- Select a map from the drop down menu. If none is available, [this guide](https://www.lgsvlsimulator.com/docs/maps-tab/#where-to-find-maps) shall be followed. To solve it, we will add a new map from the *Maps* tab by chosing a name and pasting the URL of the *Asset Bundle* hyper-reference of one of the [provided maps](https://content.lgsvlsimulator.com/maps).
- Select the *Lexus2016RXHybrid* from the drop down menu. Again, if none is available, [this guide](https://www.lgsvlsimulator.com/docs/vehicles-tab) will help. As explained in the documentation, it will be necessary to add the asset URL of the car, chose the *ROS2 Native* bridge type and add the content of *lgsvl-sensors.json* (in the root of the AutowareAuto repository) in the *sensors* box.
- Enter *127.0.0.1:9090* for the deffault setting in the *bridge connection* box.
- Click the *submit* button.


The configuration also allows further configurations regarding traffic and weather conditions. Once the simulator itself is opened, the button on the bottom left will open a window with all the controls and shorcuts (the basics is: arrows for steering, accelerating and braking).

Once the simulator is opened, it will be posible to access the ROS messages that are being published from another terminal session (from inside the ADE environment). It is possible to check that everything is woking by listing the active topics (`ade$ ros2 topic list`).

Once having all the data available, it is now possible to use the previously explained perception pipeline. To do so, a default demo will be used by executing:

```bash
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch autoware_demos lidar_bounding_boxes_lgsvl.launch.py
```

This launch file will run all the nodes of the perception stack and open RViz to visualize the ego vehicle, the frames, the LiDAR data and the bounding boxes. It is possible to remove the ground points choosing which point clouds to show. It is possible to use the ROS2 tools to see which nodes and topics are active after using the launch file.

```
NOTE: It is essential to make sure the packages are built (executing 'colcon build' from the root of the cloned repository). The building of 'lanelet2' metapackage may fail. It can be solved by leaving in all the packages it contains 'catkin' as its only build dependency (commenting the other one in their 'package.xml' files).
```

