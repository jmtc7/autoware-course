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
- Brief introduction to ROS
- Core ROS concepts
- Environment setup
- Colcon nomenclature

### [History of ROS](https://youtu.be/FTA4Ia2vLS8?t=525)



## 2.3. Nodes and Publishers
- Overview o ftopics
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


