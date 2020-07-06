# Lecture 01: Development Environment
[![Autoware.Auto badge](https://img.shields.io/badge/Autoware-Auto-orange.svg)](https://www.autoware.auto)

The lecturers are [Dejan Pangercic](https://www.linkedin.com/in/dejanpan) (ROS 2 and Autoware Developer and CTO of Apex.AI) and [Tobias Augspurger](https://www.linkedin.com/in/tobias-augspurger-5116b6191) (Product Owner at StreetScooter). The lecture video can be accessed from the following link:

[![Lecture video](https://img.youtube.com/vi/XTmlhvlmcf8/0.jpg)](https://www.youtube.com/watch?v=XTmlhvlmcf8&list=PLL57Sz4fhxLpCXgN0lvCF7aHAlRA5FoFr&index=1)

The content of this lecture will be:

- The goals of the course
- Why is it a relevant course
- Logistics and Content
- ADE Environment
- Development Strategies for Safety-Critical Systems

There are provided materials for both the [first part](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/01_DevelopmentEnvironment/devenv.md) and the [second part](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/01_DevelopmentEnvironment/devenv2.md) of the lecture, consisting in MarkDown files with the lecture contents and their PDF versions, which are included in the *resources* folder of this directory.

## Section 1. Introduction
## [1.1. Course Introduction](https://youtu.be/XTmlhvlmcf8?t=75)
Currently (May 2020), Autoware.Auto has full **localization** capabilities using LIDAR and GPS, full **perception** of other traffic participants in 2D and 3D (it can classify, infer their velocities and intended path), **motion planning** for relatively simple maneuvers and **information about the environment** in segmented semantic maps that highlight, for example, lanes, crosswalsk, traffic lights, etc. In [this video](https://www.youtube.com/watch?v=kn2bIU_g0oY), some of the Autoware capabilities can be appreciated, such as the car reacting to unexpected situations (obstacle appearing in the road), waiting for pedestrians to go across crosswalks, following a slow vehicle, backward driving and localization.

All the content will be hosted in the [Apex.AI website](https://www.apex.ai/autoware-course), where links to the videos and materials can be found, as well as a very complete syllabus of the course and the sections of each lecture.	 Every Monday a new lecture will be published until the 14 lectures are completed.

### [Importance of Autonomous Systems](https://youtu.be/XTmlhvlmcf8?t=75)
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


## [1.2. How will the course work?](https://youtu.be/XTmlhvlmcf8?t=795)
This is not a course for beginners, so there are a few assumed prerequisites, such as:

- Familiarity with ROS 1 and intermediate C++.
- Knowledge of linear algebra, calculus and statistics, specially the mathematical basis of perception, localization, planning, control, state estimation, and decision making, at least at the level taught in the Udacity's [Self-Driving Car Engineer ND](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013).
- Hardware over the [requirements](https://www.lgsvlsimulator.com/docs/faq/#what-are-the-recommended-system-specs-what-are-the-minimum-required-system-specs) imposed by the Unity-based simulator (LGSVL).
- Ubuntu 18.04 as the OS for developing and Docker Engine.

The course will advance keeping an Autonomous Valet Parking problem in mind, which will be used as the goal of the development that will be done during this course.


## [1.3. Development Environment](https://youtu.be/XTmlhvlmcf8?t=1379)
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

If the host system has a NVIDIA GPU installed, appart of its driver, it will be necessary to install **NVIDIA Docker**, as explained in its [official repository](https://github.com/NVIDIA/nvidia-docker). The commands to execute to install it in Ubuntu/Debian systems are:

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


## [1.4. Object Detection Demo](https://youtu.be/XTmlhvlmcf8?t=2168)
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

The result of this, without including the two last steps for clarity, should be a visualization of the robot (Lexus RX 450h), the LIDAR sensors, their frames, and the raw LIDAR data converted to point clouds, as can be appreciated in the next GIF (it is also a link to the full 8 minutes demo, hosted in YouTube:

[![Demo visualization](resources/lidar_visualization.gif)](https://www.youtube.com/watch?v=Ooxcm8KTMS8&list=PLBYQePjTMEGHIbWg2AF7mJLnxNA9-PSab&index=1)


### [Editing and Compiling Code](https://youtu.be/XTmlhvlmcf8?t=2550)
The first thing, if this was not already dont, is to enter the ADE environment, going into the `AutowareAuto/src` directory and sourcing the built version of Autoware.Auto:

```bash
$ ade start
$ ade enter
ade$ cd AutowareAuto/src/
ade$ source /opt/AutowareAuto/setup.bash
```

Next, the `autoware_auto_create_pkg` script, it is possible to create a new ROS 2 package in AutowareAuto with all the necessary details, such as manteinter, email, package description, etc. A sample execution of this with my data would be:

```bash
ade$ autoware_auto_create_pkg --destination . --pkg-name autoware_my_first_pkg --maintainer "Jose Miguel TORRES CAMARA" --email josemigueltorrescamara@gmail.com --description "My first Autoware pkg."
```

Now, a new folder named `autoware_my_first_pkg` should have been created and should be containing the following files and sub-directories:

- **CMakeLists.txt**: Describes how to build the code withing the package.
- **design/**: Folder for design documents, used to give guidance to the architecture of the package.
- **include/**: Headers that will be included in the source codes.
- **package.xml**: Meta information about the package.
- **src/**: Contains the source code files.
- **test/**: Testing files.

Now, as a test, we can now edit the `autoware_my_first_pkg_node.cpp` file, located inside the `src/` directory of the package that has just been created. Here are some examples on how to do this from the command line using Vim or Emacs:

```bash
ade$ vim autoware_my_first_pkg/src/autoware_my_first_pkg_node.cpp
ade$ emacs -nw autoware_my_first_pkg/src/autoware_my_first_pkg_node.cpp
```

And add a line, such as an error message in the `print_hello()` function. Finally, in order to compile the package and execute this node, these are the steps to follow:

```bash
ade$ cd .. # Move to the root of the AutowareAuto directory
ade$ colcon build --packages-select autoware_auto_autoware_my_first_pkg
ade$ source install/setup.bash
ade$ ros2 run autoware_auto_autoware_my_first_pkg autoware_my_first_pkg_exe
```




---


## Section 2. Safety
## [1.5. Safety Lecture Overview](https://youtu.be/XTmlhvlmcf8?t=2800)
This section will be covered by Tobias Augspurger, Product Owner at StreetScooter, a smaller company of DHL. They build yard logistic robots for the DHL parcel centers. The section will be about the development of safe and reliable robots for close or public roads. The official course notes can be found in this [Apex.AI repository](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/01_DevelopmentEnvironment/devenv2.md).

## [1.6. Theory Behind Developing Safety-Critical Software](https://youtu.be/XTmlhvlmcf8?t=2842)
### [Safety and Security in Automotive](https://youtu.be/XTmlhvlmcf8?t=2842)
*Safety* is the condition of being unlikely to cause danger, risk or injuries. Therefore, the safest vehicle is one that does nothing. This is why the *freedom* that is wanted to be given to the vehicle should be taken into consideration, however, it will add risks.

Given how critical are the applications related to autonomous vehicles, it is very important to develop in an environment that allows and encourage developers to talk freely about the risks they see in order to solve the maximum possible ammount of them.

### [Responsabilities and Safety Verification](https://youtu.be/XTmlhvlmcf8?t=2995)
When using open source software in comercial systems, the developers of the open source software will not usually have any responsability about what happens with the system (depending on the license terms, Autoware.Auto uses the Apache License Vesion 2.0). The manufacturer using this software is responsible of testing and validating it to certify that anything will happen. To avoid this, BMW CarIT GmbH developed a open source software [safety verification methods](http://www.bmw-carit.de/downloads/presentations/2018-04-25_Bulwahn_Linux-Safety-Verification.pdf) for the Linux kernel. This is also done in the [ELISA](https://www.linuxfoundation.org/press-release/2019/02/the-linux-foundation-launches-elisa-project-enabling-linux-in-safety-critical-systems/) project.

The [ISO 26262](https://www.iso.org/obp/ui/#iso:std:iso:26262:-2:ed-2:v1:en) enforces companies to design development processes following the mentioned standard. In order to work profesionally in this field, both the product development and the supply chain must be [certified](https://www.tuv.com/world/en/evaluation-of-supplier.html) by the [ISO 26262](https://www.iso.org/obp/ui/#iso:std:iso:26262:-2:ed-2:v1:en), which will protect the certified companies in terms of malfunctions damaging persons or objects because *[reasonable car](https://www.eejournal.com/article/20161116-liability/)* was shown.

### [Popular Software Development Models](https://youtu.be/XTmlhvlmcf8?t=3140)
There are several software development models that are commonly found these days. The main three ones are:

- **Waterfall**: One of the oldest ways to develop. It separates the development into different sequential steps that are not iterable (requirements, design, implementation, verification, and maintenance). The team will not be able to go from one step to another behind it. This model can be useful in projects with very costful changes. However, whenever the requirements are no perfectly known or they change during the development, the product can end up being useless.
- **V-Model**: Describes the relationship between the requirements and the design and verification phases. In verification processes, it is possible to show that every requirement was checked and verified using tests at the end of the development. In test-driven development, the tests are defined even before the actual implementation, so that the development will be done only to suit the requirements (represented as tests). ISO 26262 requires an adaption of this development model.
- **Agile**: There are many Agile methods, such as [Scrum](https://www.scrumguides.org/docs/scrumguide/v2017/2017-Scrum-Guide-US.pdf) or [Extreme Programming](http://www.extremeprogramming.org/), which allow iterations in the development phases, making them very flexible and useful to start working before knowing exactly what does the customer wants and shape the final product during the whole development using feedback and finding knowledge during the process.

The **V-Model for Autonomous Driving** specified by ISO 26262 starts creating the **requirements** using a hazard analysis and users specifications, considering at the same time how the **validation** will be done in order to check this. Every step, going from the *requirements* to the *module realisation*, must be traceable and validated or verified. That is why it is key to be able to demonstrate how, e.g. the system design is tested/verified with the system validation. It is not possible to create a system and use it in the development if the tests for this system are not defined (and so, it have been proven that the system can be verified).

The complexity (and almost infeasibility) of specifying all the hazards concering an autonomous car in an urban scenario is obvious, specially from the start of the development. i.e. even if all the user/customer specifications are known, the hazard analysis can not be completed. This is why this model, even it is used in the Automotive sector, can not be used by itself for high-level autonomous cars. A way of solving this can be **combining the V-Model with Agile development**, keeping traceability, validation, and verification, but being able to change requirements when needed. This combined system is called ***[Agile System Engineering](https://assets.vector.com/cms/content/consulting/publications/AgileSystemsEngineering_Vector_Ford.pdf)*** and has been done by Vector at Ford. It consists in making several V-Model developments so that, even if their outputs (*module implementations*) are failures, the gathered knowledge can be used in the beginning (*requirements*) of the next cycle to get a better version. This will be repeated again and again until a V-Model succeeds producing a complete and safe product.

This can combine with the **Operational Design Domain** (ODD), defined by J3016, which does not start with requirements, the development will begin limitating the environment or the application itself (e.g. limitate the road environment, state and behavior of the vehicle, time-of-day, traffic congestion, etc.). This has also been adopted by Autoware.Auto in a way that is explained more in-depth [here](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/01_DevelopmentEnvironment/devenv2.md). They re-used the [safety documents of the Autonomous Valid Parking (AVP) project of Parkopedia](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/206) for their own AVP project.

To sum up, the full development model will combine the Agile loops and the V-Model with the Operational Design Domain to combine the advantages of all of those. i.e. a *Safe Prototype Development Environment* will ve available for testing the new developments, technologies and solutions. At the same time, there will be an *Automated and Certified Continuous Engineering* that will allow to release the software in a certifiable and traceable way. Once a release is available, data will be gathered using it and the *Operational Design Domain* used to work at so that new V-Models can be started using the new information (which will most likely introduce new requirements) and maybe a new less restrictive ODD.

### [Separation of Concerns](https://youtu.be/XTmlhvlmcf8?t=3980)
Since the target of changing the used Operational Design Domain is to increase the functionality of the system (e.g. make it robuts to driving in different times of the day, with other vehicles in the road or in its lane, etc.) to keep iterating and generating new, safer, and more complete releases, the ***Separation of Concerns*** (SoC) comes into play. A program emboding SoC properly would be a modular program. ROS 1 and ROS 2 encourage SoC because they allow the creation of many *modules* (nodes) that communicate between eachother over a [middleware](https://design.ros2.org/articles/ros_middleware_interface.html). This allows to re-use functionalities between different Design Domains, making the development of new releases quicker and cheaper in many senses.


## [1.7. Safety-Critical Software Design in Practice](https://youtu.be/XTmlhvlmcf8?t=4060)
This section will go through how to implement the theory explained in the last section. Some of it is used in Autoware.Auto, while some parts are not (yet). However, since the process is iterative and Autoware.Auto allows gathering knowledge, this is OK as long as it is assumed that it is not finished, so it will be possible to introduce these missing parts on future cycles.

### [Sample General Design Guidelines](https://youtu.be/XTmlhvlmcf8?t=4060)
As an example, these are the Apex.AI's internal **General Design Guidelines** (which are based on [JSF-AV-rules](http://www.stroustrup.com/JSF-AV-rules.pdf)), sorted from more to less important: 

- **Reliability**: Fufilling requirements in a predictable manner.
- **Portability** of the source code: Make it not dependent on the compiler or linker.
- **Maintainability**: Make source code consistent, readable, simlpe and easy to debug.
- **Testability** is kept in mind when developing source code.
- **Re-usability** of components of the source code, reducing costs of development and testing.
- **Extensibility**: Requirements can evolve over the life of a product, so the code should be able to handle these modifications by local extensions rather than big modifications.
- **Readability**: The code must be easy to read, understand and comprehend.

### [Develop in a Fork](https://youtu.be/XTmlhvlmcf8?t=4480)
In order to develop for Autoware.Auto, the [fork-and-pull model](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/developer-guidelines.html) should be used as the contribution workflow. This consists on forking the current development, adding the development in the forked repository and finally open a *merge (or pull) request* to integrate it in the main Autoware.Auto repository. All the design guidelines are also exposed to be kept in mind while developing.

### [Designs and Requirements](https://youtu.be/XTmlhvlmcf8?t=4575)
As an example, Christopher Ho (of Apex.AI) showed in two posts ([part 1](https://www.apex.ai/post/building-safe-algorithms-in-the-open-part-1-design) and [part 2](https://www.apex.ai/post/building-safe-algorithms-in-the-open-part-2-implementation)) how they created the requirements for the NDT development, an algorithm for LIDAR-based localization and point cloud registration. A relevant part of it is the view of the integration part. It should be done keeping a high-level view while fixing low-level problems. It is very convinient to advance as much as possible in integration because, even if the parts to integrate are perfect, if they do not integrate well with each other, they will be useless and it will imply a lot of cost to re-do them so that they can be properly integrated.

This problem can be overcomed by using  **[Test-Driven Development](https://en.wikipedia.org/wiki/Test-driven_development) processes**. They propose not to develop anything that has not a test already created. That implies that nothing will be developed if it is not to fulfill a requirement.

### [Verification](https://youtu.be/XTmlhvlmcf8?t=4735)
In order to compare the exposed theory with its implementation in Autoware.Auto. In the V-Model, the final step (implementation) will be done keeping the stablished coding standard/design guidelines. The component verification will be done by unit testing. The subsystems verification is done with integration tests. Finally, the general system validation will be done using the defined Operational Design Domain.


#### [Unit Testing and Structural Code Coverage](https://youtu.be/XTmlhvlmcf8?t=4825)
**Unit testing** is the lowest level testing. It consists in creating a test for every function. It uses a groundtruth that tells when an input (or set of them) produces the expected output (or set of them). It will compare the software-produced outputs with the mentioned groudntruth examples. This tests will be ran every time that the software is modified to make sure that the change did not introduce any bug. This will also make it easier to detect the presence of [Heisenbugs](https://en.wikipedia.org/wiki/Heisenbug) to remove the parts of the code that introduce undefined behaviour.

Regarding **Structural Code Coverage**, it is a measure of how many areas of the code are used during testing, which helps to make sure that no release contains untested code.

Autoware.Auto provides a *How to* on [writing tests and measure coverages](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/how-to-write-tests-and-measure-coverage.html).

#### [Integration Testing](https://youtu.be/XTmlhvlmcf8?t=4955)
It is a hard part, since before there were several persons working on their own but now everything must work together. They should be done as early as possible in order to avoid useless efforts developing things that will not be able to be integrated. The integration tests are usually ran in simulators, such as Gazebo, [Ignition Robotics](https://ignitionrobotics.org/) or LGSVL (depending on the used sensors, more or less sophisticated simulators may be needed. e.g. if an algorithm is based on camera data, a very realistic one will be needed). This is a huge advantage because it will fulfill some of the requirements very early in the development process.

Autoware.Auto also provides guidelines on [how to create integration tests](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/integration-testing.html).

#### [Validation by Operational Design Domain](https://youtu.be/XTmlhvlmcf8?t=5290)
The validation stands in the upper part of the V-Model. Its purpose is to check if what has been built is what was initially required. This is the reason Autoware.Auto is validated by the Operational Design Domain (ODD). The development is done following a secuence of achievable technological [milestones](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones), each one of them being a different (and, most likely, more complex) ODD. This means that each milestone will require the system to be able to success doing a certain task in certain conditions (with some assumptions and restrictions). 

All this happens into a loop in which the next ODD is chosen, then defined and finally the required scenarios to support it are specified. Next, the loop starts again. This makes sure that everything that is validated could be defined and specified previously, so everything will be clear and limited to which is needed with no ambiguities.

### [Continuous Integration and DevOps](https://youtu.be/XTmlhvlmcf8?t=5390)
*Continuous Integration* (CI) is a way to automate software development so developers can get information from others without much communication between them, making the overall process quicker and more precise. A common testing and build environment will be used to avoid problems derived from the configurations of the developers' local systems.

This practice of CI is extended in the so called *DevOps*, which is a set of practices combining software development (Dev) and information-technology (IT) operations (Ops) aimed to make the development shorter and provide continuous deliverables.


