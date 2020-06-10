# Lecture 05: Autonomous Driving Stack
[![Autoware.Auto badge](https://img.shields.io/badge/Autoware-Auto-orange.svg)](https://www.autoware.auto)

This lecture is provided by [Daniel Watzenig](https://www.linkedin.com/in/daniel-watzenig-4a0b985) and by [Markus Schratter](https://www.linkedin.com/in/markusschratter), from Autonomous Stuff. The lecture is available in YouTube:

[![Lecture video](https://img.youtube.com/vi/nTI4fnn2tuU/0.jpg)](https://www.youtube.com/watch?v=nTI4fnn2tuU&list=PLL57Sz4fhxLpCXgN0lvCF7aHAlRA5FoFr&index=5)

The lecture will go through the motivation to use Autonomous Driving (AD) Stacks, the architecture of the AD stack based on Autoware, other AD stacks, how to integrate Autoware in research vehicles and an Autoware use case. In general, a view of the AD stacks will be provided. i.e. the required building blocks to create a self-driving car product that is safe to sell. The main concepts of the lecture will be:

- What are the AD stacks.
- Why they are necessary.
- How to use the Autoware-provided AD stack.

The provided PDFs can be found in the Apex.AI's [autowareclass2020 repository](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/05_Architectures), in GitLab. There is one for the [first](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/05_Architectures/Part_1.pdf), [second](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/05_Architectures/Part_2.pdf) and [third](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/05_Architectures/Part_3.pdf) parts of the lecture. The first part will go through AD stacks, the second will be about why and how to integrate Autoware in a research vehile and the third will analyze an Autoware use case.


---

## First part
## [5.1. Autonomous Driving Stacks](https://youtu.be/nTI4fnn2tuU?t=45)
During this first part, the hihg level architecture of Autonomous Vehicles (AVs) will be introduced, as well as the signal flow and timing requirements. The control layer will be seen on more depth, as well as the difference between concepts such as *safety critical*, *mission critical*, *load critical* and *mixed critical* systems.

### 5.1.1. Architecture of AD Stack
### 5.1.2. Building AD Blocks
- Sensors
- Actuators
- Perception
- Localization
- Map
- Planning
### 5.1.3. Signal Flow and Block Interaction
### 5.1.4. Control Layer Decomposition
- Active Safety 0.1-15 s
- Strategic/Mission Level 10+ s
- Tactical level 1-10 s
- Reactive/Operational Level 0.01-0.1 s

safety-critical vs mission-critical vs low-critical vs mixed criticality systems (MCSs)
### 5.1.5. Autoware
- Autoware.AI
- Autoware.Auto
### 5.1.7. Other AD Stacks
- NVidia DriveWorks and Drive Constellation Architecture
- Apollo Architecture and Modules


---

## Second part
## 5.4. Integration of Autoware.AI in a Research Vehicle
### 5.4.1. What is Needed?
### 5.4.2. Hardware and Integration Overview
### 5.4.3. Providing AD Functionality for Research Projects


---

## Third part
## 5.5. Use Case: Roborace
- Using Autoware components
