# Lecture 06: Autoware 101: History and Future of The Autoware Foundation
[![Autoware.Auto badge](https://img.shields.io/badge/Autoware-Auto-orange.svg)](https://www.autoware.auto)

This lecture is provided by [Josh Whitley](https://www.linkedin.com/in/maximus5684), Software Architect at the Autoware Foundation. The lecture is available in YouTube:

[![Lecture video](https://img.youtube.com/vi/eSHHmJrqpMU/0.jpg)](https://www.youtube.com/watch?v=eSHHmJrqpMU&list=PLL57Sz4fhxLpCXgN0lvCF7aHAlRA5FoFr&index=6)

The lecture will be about The Autoware Foundation and its to primary projects: Autoware.AI and Autoware.Auto.

The provided PDFs can be found in the Apex.AI's [autowareclass2020 repository](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/06_Autoware_101/Autoware_101.pdf), in GitLab. The PDF is included in the *resources* folder of this directory.

## [6.1. The Autoware Foundation Structure](https://youtu.be/eSHHmJrqpMU?t=70)
Autoware.AI was started in 2015 by Shinpei Kato at Nagoya University and now (2020) is supported by the largest AD open source community while being used by more than a 100 companies and run on more than 30 vehicles in more than 20 countries. Autoware is used by automotive OEMs to develop Mobility as a Service (MaaS) and has been qualified to run on AVs on public japanese roads since 2017. More information can be found in the [Autoware Foundation website](https://www.autoware.org).

The foundation's structure starts with the board of directors, which oversees the technical steering comittee, formed by premium members and invited guests. This comittee decides which Operational Design Domains (ODDs) will be developed next and the projects of the foundation. All members compose the task forces and the work groups, while being overseen by the technical steering comittee. Those work groups and task forces are open to the public, so anyone from the community can join their calls. 

### [6.1.1. Projects of The Autoware Foundation](https://youtu.be/eSHHmJrqpMU?t=290)
The Autoware Foundation is mainly working on three projects:
- **Autoware.IO**: Its target is to make it possible to run Autoware software in as much hardware (HW) as possible, which includes the creation of reference designs for new HW builds, development of simulators and other tools, ease the sensor and ECU integration, etc. Further information can be found in [autoware.io](https://www.autoware.io), the intentions of the autoware reference platform is in this [ROS Discourse thread](https://discourse.ros.org/t/wg-rp-autoware-reference-platform-definition-documentation/9949), the first open designed hardware platform with available BSP is in this other [ROS Discourse](https://discourse.ros.org/t/open-source-and-free-software-for-autocores-pcu/12418) and, finally, the HW documentation is on [Autocore.AI's GitHub](https://github.com/autocore-ai/autocore_pcu_doc). The current status is the availability of the first board using this project, which is provided by [96Boards](https://www.96boards.org), and sensor integration. The intent is to have multiple vehicle integration, an established test framework and multiple SoC platforms (appart of the 96Boards one).
- **Autoware.AI**: 
- **Autoware.Auto**: 


## [6.2. History, Present and Future of Autoware.AI](https://youtu.be/eSHHmJrqpMU?t=545)
It is intended to be a full self-driving stack (between SAE levels 3 and 4). It is based on ROS 1, supports many vehicle platforms and contains several useful files and examples to facilitate the simulations and PoCs. More information can be found in [autoware.ai](https://www.autoware.ai), the code is available in [GitHub](https://github.com/Autoware-AI/autoware.ai), where the documentation can be also found, under the [wiki tab](https://github.com/Autoware-AI/autoware.ai/wiki).

Regarding its **history**, it was started in 2015 by Shinpei Kato at CMU (being the beginning of the Autoware Foundation), including several features developed by students at Nagoya University (Japan). After the first release in 2015, Tier IV provided most of the development until 2018. It was moved to GitLab in 2019 but returned to GitHub in 2020.

Its **current status** is supporting ROS Melodic and there is no intention to support ROS Neotic. It contains implementation of algorithms in control, localization, perception, planning and simulation and supports multiplu simulators, such as LGSVL and Carla. The Autonomous Stuff's [Open Autonomy Pilot](https://autonomoustuff.com/services/open-autonomy-pilot) is an example of a commercial service consisting in a modified version of Autoware.AI. Since it was not designed for functional safety, the way it was implemented back in the time did not has code quality and development best practices as priorities because it was conceived for R&D and testing rather than for real-world applications.

Finally, about its **future**, the last major release will be the 1.15.0, scheduled for December 2020. From this point until December 2022 it will only be maintained, date in which the project will be shutted down to focus more in Autoware.Auto.

## [6.3. History, Present and Future of Autoware.Auto](https://youtu.be/eSHHmJrqpMU?t=845)
It is also an open source Autonomous Vehicle (AV) stack, but it is based on ROS 2 and, therefore, uses the DDS middleware. Its development is done so it will support increassingly more complex Operational Design Domains (ODDs) and its modular API makes it very easy to integrate (potentially proprietary) extensions. Usage and documentation can be found in the [Autoware.Auto's GitLab](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto) and more information in the [Autoware.Auto webiste](https://www.autoware.auto). The Autoware.Auto development was done with much more *certifiable* and *industrial* values in mind, such as:

- Safety
- Code quality
- Static memory allocation
- Documentation
- Modularity


About its **history**, its development began in August 2018, mainly by Apex.AI with Tier IV. The first release (0.0.1) was in August 2019 and, currently, most of the members of the Autoware Foundation (AWF) are active on its development, which is managed by the Autoware Software Working Group (ASWG).

The **current status** is targeting ROS 2 Dashing, but it is planned to support ROS 2 Foxy after the AVP release. It is also based in Docker, using ADE as the development environment, increasing the repeatability and easiness of usage. The Docker containers natively support amd64 and arm64. The first and current ODD is Autonomous Valet Parking (AVP). Autoware.Auto has not reached yet the amount of features present in Autoware.AI, but offers the following:

- **Sensing**: Even ROS drivers and messages can be used, Autoware.Auto supports specifically the following sensors:
  - PointGrey and FLIR cameras
  - Xsens GPS and IMUs
  - Velodyne Puck, Puck LITE and Puck Hi-Res sensors
- **Localization**: NDT Matching
- **Ground Filtering**: Ray Classifier.
- **Object Detectoin**: With LiDAR data and voxel grid or euclidean clustering.
- **Vehicle Interface**: Currently it is done through Linux SocketCAN or with interfaces specific for each simulator.
- **Motion Control**: Pure pursuit and MPC.

Regarding the near **future** of Autoware.Auto, after the current 3rd milestone in the current ODD, the things to come are:

- **4th milestone**: On-vehicle integration consisting on switching from simulation to real vehicle (configure HW, add drivers and interfaces, etc.).
- **5th milestone**: AVP demo, which will consist on a week-long hackathon after which it is planned to have a mobile app to interact with the vehicle.
- **1st clean-up milestone**: After the AVP ODD is completed, missing documentation will be added and best-practices that were not taken into account before will be implemented in the already finished code.
- **Move back to GitHub**, as it was done with Autoware.AI.


## [6.4. Autoware.Auto's Development Process, Contributions and Support](https://youtu.be/eSHHmJrqpMU?t=1290)
### [6.4.1. ODD-Based Development Cycle](https://youtu.be/eSHHmJrqpMU?t=1300)
The ODD definition process starts with the AWF's Technical Steering Committee defining the ODD to target. Next, the ODD Working Group (yet to be formed) specifies the scenarios required to support the specified ODD. Next, the Technical Steering Committee chooses the next ODD and so on. While a new ODD is being chosen, the previous one will be developed by the HW and SW development groups.

### [6.4.2. How to Contribute](https://youtu.be/eSHHmJrqpMU?t=1395)
In order to contribute to the AWF's projects, these are the things to do:

- Follow the development on [GitLab](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues).
- Learn about contributing to open source with this [GitHub guide](https://github.com/firstcontributions/first-contributions).
- Join the [Autoware Slack](http://autoware.herokuapp.com).
- Talk to the current contributors on [ROS Discurse](https://discourse.ros.org/c/autoware).
- Join the Foundation (as a member organization) emailing `auto@autoware.org`.

### [6.4.3. Where to Get Support](https://youtu.be/eSHHmJrqpMU?t=1465)
The first place to search for support is the documentation. There are specific versions for [Autoware.AI](https://github.com/Autoware-AI/autoware.ai/wiki) and [Autoware.Auto](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto). The next step would be to ask in the Autoware-specific [ROS Answers page](https://answers.ros.org/questions/scope:all/sort:activity-desc/tags:autoware/page:1/). Finally, **only if it is a confirmed bug or feature request**, an issue could be filed in [Autoware.Auto's GitLab](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues).


