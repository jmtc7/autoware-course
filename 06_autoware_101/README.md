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


## 6.4. Architectural Overview of Autoware.Auto


## 6.5. Autoware.Auto's Development Process, Contributions and Support
### 6.5.1. ODD-Based Development Cycle
### 6.5.2. How to Contribute
### 6.5.3. Where to Get Support


