# Lesson 02: ROS 2 101
[![Autoware.Auto badge](https://img.shields.io/badge/Autoware-Auto-orange.svg)](https://www.autoware.auto)

This lesson will be provided by Katherine Scott, Developer Advocate at Open Robotics. The lesson video is available in YouTube:

[![Lesson video](https://img.youtube.com/vi/FTA4Ia2vLS8/0.jpg)](https://www.youtube.com/watch?v=FTA4Ia2vLS8)

The content of this lesson will be an overview of the C++ API for ROS 2. The ADE environment will be used for the lecture, which is a *dockerized* container of ROS. During the lesson, most of the ROS 2 Dashing tutorials will be installed and some of them will be explained in more detail, providing hints, tips and tricks during the process. Also, the ROS 2 CLI interface will be introduced, but the next lesson will cover it in more detail.

The provided slides in PDF format can be found in the Apex.AI's [autowareclass2020 repository](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/02_ROS2_101/lesson2.pdf), in GitLab.


## [2.1. Introduction](https://youtu.be/FTA4Ia2vLS8?t=170)
This lesson is intended to be a crash course on ROS 2 Dashing, its API and a bit in its build tools. Its target is to make the students able to code and build simple ROS 2 applications. ROS 2 Foxy will be released in a few weeks, which will be a major LTS distribution, so some things may change from Dashing to Foxy.

### [Getting Help](https://youtu.be/FTA4Ia2vLS8?t=330)
ROS has been around for over a decade right now (2010-2020), so there are a lot of useful resources to help new incomers. These resources are the following:

- **[ROS Answers](http://answers.ros.org)**: A QA website in the Stack Overflow style.
- **[ROS Discourse](https://discourse.ros.org)**: Community hub for news and discussion. NOT for asking questions.
- **[ROS Wiki](http://wiki.ros.org)**: Mostly ROS 1, but most of the content is still useful for ROS 2.
- **[ROS 2 Documentation](https://index.ros.org/doc/ros2)**: The official ROS 2 documentation, usually the first go-to. Most of this lesson comes from here, so more detailed explanations can be found in this documentation.

### [Unofficial Resources](https://youtu.be/FTA4Ia2vLS8?t=460)
There are some other unofficial resources where relevant information might be found. Those are:

- **[ROS subreddit](https://www.reddit.com/r/ROS/)**
- Unofficial **[ROS Discord](https://discord.gg/KKhcmVN)**
- **[ROSCon](https://roscon.ros.org/2020/)**, the yeraly ROS developers conference. Most of the old talks are available on the web.
- **Twitter** accounts:
  - **[@OpenRoboticsOrg](https://twitter.com/OpenRoboticsOrg)** is the most active one
  - **[@ROSOrg](https://twitter.com/rosorg)** for official ROS announcements


## [2.2. ROS Introduction](https://youtu.be/FTA4Ia2vLS8?t=525)
### [History of ROS](https://youtu.be/FTA4Ia2vLS8?t=525)
The history of ROS will be presented in order to be able to understand its structure and philosophy, seeing it keeping the context in which it was developed in mind, instead of only the modern development perspective. Back in the 2000s, Open Source was growing, but Windows dominated. Back then, *robots* were mainly robotic arms for manufacturing running over real-time (RT) control systems.

Next, in 2006 Former Google VPs decided to apply Google's knowledge in robots, creating the company *Willow Garage* with a similar spirit of those of Bell Labs or PARC (from Xerox). From this organization, things as OpenCV, PCL, ROS, PR2 robot and a lot of spin out startups were originated.

In 2012, Willow Garage folds and Open Robotics emerged from the people that were inside there and continued developing ROS. In 2017, they decided to fix all the limitations based on the feedback regarding things like *ROS is only useful for academics and research*. This was when ROS 2 was originated, with upgrades addressing security, robustness, quality of service, determinedness, etc.

### [Concepts that Motivate ROS](https://youtu.be/FTA4Ia2vLS8?t=695)
ROS's design was based by design patters that were successfully used before, being the following the correspondences between ROS concepts and the *things* they are based on:

- **ROS Nodes** - Processes and/or threads: Self-contained processes, like programs. ROS is a collection of tools that allow several programs to be executed in parallel.
- **ROS Topics** - Buses or PubSub: ROS' backbone is a publihs/subscribe bus. ROS Topics work in a similar way to ZeroMQ, RabbitMQ, or ModBus. i.e. things are published somewhere by someone and they can be read by other agents.
- **ROS Messages and Bags** - Serialization: They are the data moved over the topics, so that this data is serialized between nodes (that can be implemented in different programming languages). Google Protocol Buffers are similar. ROS Messages can be written in a file called a ROS Bag, similar to Python's Pickle files.
- **ROS Params** - Black Board Pattern: A way to create global variables between nodes/programs. Similar to Redis.
- **ROS Services** - Synchronous Remote Procedure Call (RPC): Programs that can be called by another ones. The caller will be blocked until the callee returns the answer.
- **ROS Actions** - Asynchronous Remote Procedure Call (RPC): Again, programs callable by other programs, but the caller will not be blocked until the callee returns as with the services.
- **ROS Life cycles** - State Machines: Tool to move between states. Useful for modeling behaviours.
- **URDF and TF** - Matrix Math for 3D Operations: TF stands for *Transform* and URDF for *Universal Robot Description Format*. They are tools for automatically calculating robot geometry using matrix math.

### [Jumping in the Deep End - Environment Setup](https://youtu.be/FTA4Ia2vLS8?t=1000)
After the [Lesson 01](https://github.com/jmtc7/autoware-course/tree/master/01_development_environment), ADE is already installed and working using the Autoware instructions. Next thing to do is to update the system, and install ROS Dashing and some tools. To do so, the commands to be executed are:

```bash
# Enter the environment
$ ade start
$ ade enter

# Source ROS Dashing (if it is not done in the local ~/.bashrc file)
ade$ source /opt/ros/dashing/setup.bash

# Instal Turtlesim and QT GUI apps
ade$ sudo apt update
ade$ sudo apt install ros-dashing-turtlesim
ade$ sudo apt install ros-dashing-rqt-*

# Optional for managing terminals, other options are Terminator and TMux
#ade$ sudo apt-install byobu

# Install any IDE, text editor or tools that you may want to be available in the environment
```

### [Colcon Nomenclature](https://youtu.be/FTA4Ia2vLS8?t=1140)
ROS is built upon **packages**, which are collections of code (C++ or Python mainly, but Java or Go are also possible). There are a lot of packages released on the web that can be pulled and used. A **workspace** is a collection of source code and packages oriented to be depolyed on a robot or system. A Python virtual environment could be an analogy to that. **Overlays** are second workspaces with more packages. In case of duplicity, the package or code at the bottom will be used. **Underlays** are workspace underneath an overlay, some sort of layering Python's virtual environments on top of each other. Finally, **Colcon** is the ROS 2 build tool, some sort of a layer abouve CMake or Make.

As an example, a repository will be cloned and built. In order to do this, three terminal sessions will be needed, all of them inside the container and with ROS Dashing sourced. The steps to do this are the following:

```bash
# Create a folder to be used as the workspace and go into it
mkdir -p ~/ros2_example_ws/src/
cd ~/ros2_example_ws/

# Clone the ROS 2 examples
git clone https://github.com/ros2/examples src/examples/

# Check the examples for the Dashing distribution
cd ~/ros2_example_ws/src/examples
git checkout dashing

# Build the examples
cd ~/ros2_example_ws/
colcon build --symlink-install
```


## [2.3. Nodes and Publishers](https://youtu.be/FTA4Ia2vLS8?t=1485)
- Overview of topics
- Building and running a node
- Simple publisher build and run
- Modifying the publisher
- Building a subscriber 
- Pub/sub working together


## 2.4. Services
- Concept overview
- Review basic service
- Running basic services
- Calling services from the command line
- Building a service client
- Executing a server/client service


## 2.5. Actions
- Action Overview
- Action file review
- Basic action review
- Running/calling an action
- Action client review
- Running an action server with a client


