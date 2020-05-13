# Additional Information
In this folder, additional relevant information related to the course, ROS 2 and Autoware is contained.


## Introduction to Autoware. Why Autoware.Auto?
Autoware is a software stack maintained by [The Autoware Foundation](https://www.autoware.org/), which goal is to support open-source projects that will enable self-driving mobility. It has many members from both industrial and academical environments. In 2020, there are thee Autoware projects, which are:

- [**Autoware.AI**](http://www.autoware.ai/): It is based in ROS 1. It was the first Autoware project and its goal is to be a R&D platform for autonomous driving. Its main use is in academia.
- [**Autoware.IO**](http://www.autoware.io/):: An interface for Autoware to be extended with proprietary software and third-party libraries. It contains sensor drivers, by-wire controllers, hardware-dependent programs for System on Chip (SoC) boards, etc.
- [**Autoware.Auto**](http://www.autoware.auto/):: Somehow an evolution of Autoware.ai. It is based in ROS 2, which gives it Real-Time (RT) capabilities and more security measures that makes it more trustable for real world critical applications. It is being freshly developed to make sure that the best development practices are used, so it will not only be a quick adaptation of Autoware.ai. Further details on how this migration from ROS 1 to ROS 2 is done can be found [here](https://www.apex.ai/post/porting-algorithms-from-ros-1-to-ros-2). The following are some relevant concepts kept in mind during the migration:
  - Usage of the **mulithreading** only when it is needed, but in every case it will be useful.
  - Interruption-based **polling** instead of infinite polling to avoid jitter.
  - Keep the **concers separated**. In particular, they will be (as suggested [here](https://limo.libis.be/primo-explore/fulldisplay?docid=LIRIAS1748457&context=L&vid=Lirias&search_scope=Lirias&tab=default_tab&lang=en_US&fromSitemap=1): computation, communication, coordination, composition and configuration. Even some of those are a ROS task, the developers have kept it in mind to separate the concers in their power.
  - **Safety-critical coding** standards linked with data and time deterministic systems. Some of the existing standards are: [Autosar Standard](https://www.autosar.org/fileadmin/user_upload/standards/adaptive/17-10/AUTOSAR_RS_CPP14Guidelines.pdf), [MISRA C/C++ Standard](https://www.misra.org.uk/Activities/MISRAC/tabid/171/Default.aspx) and [JSF-AV](http://www.stroustrup.com/JSF-AV-rules.pdf).
  - Avoiding **dynamic memory allocation** to avoid potential runtime problems.
  - Avoiding **blocking calls** so that they will not make the application undeterministic.
  - **Error handling and reporting**.
  - **Unit testing** ov every function, line, branch, and condition of the code to ensure its correctness.


