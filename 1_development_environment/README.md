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


## [1.2.1. Quick Start of the Development Environment](https://youtu.be/XTmlhvlmcf8?t=1379)

---

## Section 2. Safety
## 1.2.1. Development of Complex and Safety-Critical Software - The Theory


## 1.2.2. Development of Complex and Safety-Critical Software - The Practice


---


## Section 2. Conclusions and the Next Lecutre

