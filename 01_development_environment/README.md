# Lesson 1: Development Environment
The lecturers are (1) Dejan Pangercic (ROS 2 and Autoware Developer and CTO of Apex.AI) and Tobias Augspurger (StreetScooter). The lesson video can be accessed from the following link:

[![Lesson video](https://img.youtube.com/vi/XTmlhvlmcf8/0.jpg)](https://www.youtube.com/watch?v=XTmlhvlmcf8)

The content of this lesson will be:

- The goals of the course
- Why is it a relevant course
- Logistics and Content
- ADE Environment
- Best Practices for Autonomous Systems Applications

There are provided materials for both the [first part](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/01_DevelopmentEnvironment/devenv.md) and the [second part](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/01_DevelopmentEnvironment/devenv2.md) of the lesson, consisting in MarkDown files with the lesson contents.

## Section 1. Introduction
## [1.1.1. Course Introduction](https://youtu.be/XTmlhvlmcf8?t=75)
Currently (May 2020), Autoware.Auto has full **localization** capabilities using LIDAR and GPS, full **perception** of other traffic participants in 2D and 3D (it can classify, infer their velocities and intended path), **motion planning** for relatively simple maneuvers and **information about the environment** in segmented semantic maps that highlight, for example, lanes, crosswalsk, traffic lights, etc. In [this video](https://www.youtube.com/watch?v=kn2bIU_g0oY), some of the Autoware capabilities can be appreciated, such as the car reacting to unexpected situations (obstacle appearing in the road), waiting for pedestrians to go across crosswalks, following a slow vehicle, backward driving and localization.

All the content will be hosted in the [Apex.AI website](https://www.apex.ai/autoware-course), where links to the videos and materials can be found, as well as a very complete syllabus of the course and the sections of each lesson.	 Every Monday a new lecture will be published until the 14 lessons are completed.

### Importance of Autonomous Systems
It is a **megatrend**, as can be clearly appreciated by the increase of investments in this field, which has gone from 0.6 to 5.6 billions of Dollars within 5 years. Another consideration is the potential improvement in both **traffic congestion and road safetyi**.

Another reason for the existance of this course is the **maturity of** the two main used pieces of software: **ROS 2 and Autoware**. They are also relatively new technologies that not many people are familiarized with.

Regarding the implementation, almost every company working with Autonomous Systems uses **ROS** in one way or another. This software has been growing a stablishing itself during the last 10 years and, specially with ROS 2, it has became really hadly for developing Autonomous Systems applications. Some of the advantages of ROS 2 over ROS 1 are:

- Quality of desing and implementation.
- System reliability.
- Real-time control and deterministic execution.
- Validation, verification and certification.
- Flexibiliity in communication.
- Support for small embedded systems.

**Autoware** has been a thing since 2015, it has the support of many companies and its currently (May 2020) running in over a hundred cars.

The **complexity of developing Autonomous Vehicles** (AVs) and applications for them makes it very helpful to have already well-developed, tested, and reliable software that can be used in order to avoid re-investing resources in developing what has already been developed. This allows the end users (companies developing applications for AVs) to focus on the application itself instead of the underlying technologies.


## [1.1.2. How will the course work?](https://youtu.be/XTmlhvlmcf8?t=795)
This is not a course for beginners, so there are a few assumed prerequisites, such as:

- Familiarity with ROS 1 and intermediate C++.
- Knowledge of linear algebra, calculus and statistics, specially the mathematical basis of perception, localization, planning, control, state estimation, and decision making, at least at the level taught in the Udacity's [Self-Driving Car Engineer ND](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013).
- Hardware over the [requirements](https://www.lgsvlsimulator.com/docs/faq/#what-are-the-recommended-system-specs-what-are-the-minimum-required-system-specs) imposed by the Unity-based simulator (LGSVL).
- Ubuntu 18.04 as the OS for developing and Docker Engine.

The course will advance keeping an Autonomous Valet Parking problem in mind, which will be used as the goal of the development that will be done during this course.

### Next Lectures
Lectures 2 and 3 will be done by Katherine, from Open Robotics, and will be about ROS 2. The forth lesson will be given by AD-Link and it will be about an AV platform, composed by the hardware, a real-time operative systems (RTOS) and DDS (the middleware of choice in ROS2). The 5th lesson will be done by the Virtual Vehicle Research Center of Graz, Austria, and will be about the architecture of Autonomous Driving stacks and a bit of their history, presenting a lot of the problems of developing these systems. Next, the lesson 6, by The Autoware Foundation, will present the history and evolution of Autoware. During the lessons 7, 8 and 9, the main perception modalities will be analyzed. The 10th lesson will be about fusing data from IMUs and odometry and how to use it for localization. The lesson 11 will go over the localization module of Autoware and its application in Autonomous Valet Partking (AVP) and will be done by Autoware Foundation and Apex.AI. The lesson 12 will be given by LGE, the company behind the used LGAVL simulator, and will cover the simulation of AVs. The lesson 13 will be about HD maps (creation and usage) and will be done by Parkopedia. The 14th lesson, by Embotech, will be about motion planning and control. Finally, the lesson 15 will be about a tool called MARV, created by the company Ternaris, and will be about how to mine, store and analyze data.

Each lesson will contain a theoretical background, programatic examples and systematic examples, so that the students will be able to experiment with the *labs*.


## [1.2.1. Development Environment](https://youtu.be/XTmlhvlmcf8?t=1379)
### ADE Installation
The development environment that will be used is ADE, which is a wrapper around Docker that allows interaction with Docker (e.g. starting/stopping dockers), easy configurations and versioning of docker volumes. A guide for the installation can be found [here](https://ade-cli.readthedocs.io/en/latest/install.html#requirements), which is basically a **Docker installation followed by** the following actions:

```bash
# ADE Installation
$ cd ${HOME}
$ mkdir adehome # Create a persistent environment to store the things that must persist between ADE sessions
$ cd adehome
$ wget https://gitlab.com/ApexAI/ade-cli/uploads/85a5af81339fe55555ee412f9a3a734b/ade+x86_64 # Fetch ADE binary from GitLab
$ mv ade+x86_64 ade # Rename it to "ade"
$ chmod +x ade # Give execution rights
$ mv ade ~/.local/bin # Move it to a proper location
$ which ade # Check its location
$ ade update-cli # Update the ADE Client
```

Next, in order to enable autocompletion, the following lines should be added to the `.zshrc` or `.bashrc` file of the system:

```bash
if [ -n "$ZSH_VERSION" ]; then
    eval "$(_ADE_COMPLETE=source_zsh ade)"
else
    eval "$(_ADE_COMPLETE=source ade)"
fi
```

If the host system has a NVIDIA GPU installed, appart of its driver, it will be necessary to install **NVIDIA Docker**, as explained in its [official repository](sudo apt-install byobu). The commands to execute to install it in Ubuntu/Debian systems are:

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

```

Finally, to **setup the ADE environment**, the following steps should be followed:

```bash
# ADE Setup
$ touch .adehome # Create an empty configuration file
$ git clone --recurse-submodules https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git # Clone Autoware.Auto
$ cd AutowareAuto/
$ ade start
$ ade enter
```

NOTE: when running `ade start`, an error telling that the permisson to connect ot the Docker daemon socket has been denied. As explained [here](https://www.digitalocean.com/community/questions/how-to-fix-docker-got-permission-denied-while-trying-to-connect-to-the-docker-daemon-socket), the solution is to create a *docker* group and add the user to it. The error should be fixed after logging out and in again.

### [ROS 2](https://youtu.be/XTmlhvlmcf8?t=1875) and [Autoware.Auto](https://youtu.be/XTmlhvlmcf8?t=2070) Installations
Autoware.Auto uses **ROS 2** Dashing, which is already installed inside ADE (installed in /opt/ros/dashing/). The installation can be confirmed by running `ade$ ros2 -h`, and a basic talker/listener example can be executed by using:

```bash
ade$ ros2 run demo_nodes_cpp talker

ade$ ros2 run demo_nodes_cpp listener
```

Some additional system packages can be installed inside ADE with the `apt` package manager as follows:

```bash
ade$ sudo apt update
ade$ sudo apt install ros-dashing-turtlesim
ade$ sudo apt install ros-dashing-rqt-*
ade$ sudo apt-install byobu
```

NOTE: Between `ade stop` and `ade start`, these installations will be lost. If its persistance is important, they should be placed in the `adehome` directory.

Regarding **Autoware.Auto**, it is also already installed in the Docker image. The installation will be in `/opt/Autoware.Auto/`, which is constructed as a docker volume. To install it from source, the previously cloned version can be used, alongside with the following sequence of commands:

```bash
ade$ cd AutowareAuto
ade$ colcon build
ade$ colcon test
ade$ colcon test-result
```


## [1.3.1. Object Detection Demo](https://youtu.be/XTmlhvlmcf8?t=2168)
It is a LIDAR-based object detection demo. The first necessary thing to do is to download a pre-recorded ***[pcap file](https://drive.google.com/open?id=1vNA009j-tsVVqSeYRCKh_G_tkJQrHvP-)***, which is a collection of UDP packages recorded while driving in Palo Alto. This file should be moved into a folder named `data/`, in the `adehome` directory. Next, the **configuration files** for this lectures should be cloned from the following ApexAI repository (from insider the ADE environment):

```bash
ade$ git clone https://gitlab.com/ApexAI/autowareclass2020.git ~/autowareclass2020
```

The next step is to source the Autoware.Auto workspace before each of the commands that will be specified in the next step. This is done by executing the following in the ADE terminal:

```bash
ade$ source /opt/AutowareAuto/setup.bash
```

Once having everything setted up, it is possible to **replay the data and launch the ROS 2 nodes** that will process the input pointclouds to create 3D bounding boxes around the detected objects. These are the commands to be executed:

```bash
# Replay and broadcast the UDP data from the pcap file (adding "-r -1" will replay it in a loop)
ade$ udpreplay ~/data/route_small_loop_rw-127.0.0.1.pcap

# Launch RViz 2 to visualize the data
ade$ rviz2 -d /home/${USER}/autowareclass2020/code/src/01_DevelopmentEnvironment/aw_class2020.rviz

# Launch the Velodyne driver to convert the raw LIDAR data into pointclouds
ade$ ros2 run velodyne_node velodyne_cloud_node_exe __ns:=/lidar_front __params:=/home/${USER}/autowareclass2020/code/src/01_DevelopmentEnvironment/velodyne_node.param.yaml

# Robot State Publisher. Publishes the transforms between the coordinate system of the car and the one of the LIDARs on the roof
ade$ ros2 run robot_state_publisher robot_state_publisher /opt/AutowareAuto/share/lexus_rx_450h_description/urdf/lexus_rx_450h.urdf

# Point Cloud Transformer. Will transform the pointlouds (in the LIDARs' coordinate frames) to the base coordinate frame of the car
ade$ ros2 run point_cloud_filter_transform_nodes  point_cloud_filter_transform_node_exe __ns:=/lidar_front __params:=/opt/AutowareAuto/share/point_cloud_filter_transform_nodes/param/vlp16_sim_lexus_filter_transform.param.yaml __node:=filter_transform_vlp16_front

# Ray Ground Classifier differenciates the data corresponding to the ground from the one corresponding to objetcs
ade$ ros2 run ray_ground_classifier_nodes ray_ground_classifier_cloud_node_exe __ns:=/perception __params:=/opt/AutowareAuto/share/autoware_auto_avp_demo/param/ray_ground_classifier.param.yaml

# Convert the non-ground points into obstacles to be avoided
ade$ ros2 run  euclidean_cluster_nodes euclidean_cluster_exe __ns:=/perception __params:=/opt/AutowareAuto/share/autoware_auto_avp_demo/param/euclidean_cluster.param.yaml
```

**NOTE**: When trying to source the Autoware.Auto workspace, I got the following error, but I could execute everything without any problem:

```bash
not found: "/opt/AutowareAuto/share/spinnaker_camera_driver/local_setup.bash"
not found: "/opt/AutowareAuto/share/spinnaker_camera_node/local_setup.bash"
```


---

## Section 2. Safety
## 1.2.1. Development of Complex and Safety-Critical Software - The Theory


## 1.2.2. Development of Complex and Safety-Critical Software - The Practice


---


## Section 2. Conclusions and the Next Lecutre

