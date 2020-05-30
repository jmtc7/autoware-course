# Lecture 03: ROS 2 CLI Tooling
[![Autoware.Auto badge](https://img.shields.io/badge/Autoware-Auto-orange.svg)](https://www.autoware.auto)

This lesson will be provided by Katherine Scott, Developer Advocate at Open Robotics. The lesson video is available in YouTube:

[![Lesson video](https://img.youtube.com/vi/wcibIqiRb04/0.jpg)](https://www.youtube.com/watch?v=wcibIqiRb04&list=PLL57Sz4fhxLpCXgN0lvCF7aHAlRA5FoFr&index=3)

The content of this lesson will be an overview of the Command Line Interface (CLI) tools that ROS 2 provides for building, runing, degubing, introspecting and deploying ROS 2 systems. This lecture's motivation is to share knowledge on how to get familiar with systems that other people has developed or get insights on how our own ones are working.

The provided materials in *reStructuredText* can be found in the Apex.AI's [autowareclass2020 repository](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/03_ROS_Tooling/Lesson3CLI.rst), in GitLab. They are also able in PDF.

## [3.1. Setting Up the Environment](https://youtu.be/wcibIqiRb04?t=250)
As in the previous lecture, ROS 2 Dashing will be used, which is supported for Ubuntu 18.04 machines. Following the steps explained in the [first lecture](https://github.com/jmtc7/autoware-course/tree/master/01_development_environment), these are the commands that need to be executed before continuing:

```bash
# Start and enter the ADE environment
$ cd adehome/AutowareAuto/
$ ade start
$ ade enter

# Source the ROS installation to be used (ROS 2 Dashing)
ade$ source /opt/ros/dashing/setup.bash

# Update apt lists and install packages that will be used in this lecture
ade$ sudo apt update
ade$ sudo apt install ros-dashing-turtlesim
ade$ sudo apt install ros-dashing-rqt-.*
```

## [3.2. Basic Command Line Tooling in ROS 2](https://youtu.be/wcibIqiRb04?t=330)
### [Getting help](https://youtu.be/wcibIqiRb04?t=330)
`ros2` is an extensible command line tool. `ros2 --help` will print a list of the available commands that can be used to extend its functionality. To learn more about each of this commands, `ros2 <command> -h` should be used. The output of this command will be:

```bash
ade$ ros2 --help
usage: ros2 [-h] Call 'ros2 <command> -h' for more detailed usage. ...

ros2 is an extensible command-line tool for ROS 2.

optional arguments:
  -h, --help            show this help message and exit

Commands:
  action     Various action related sub-commands
  bag        Various rosbag related sub-commands
  component  Various component related sub-commands
  daemon     Various daemon related sub-commands
  launch     Run a launch file
  lifecycle  Various lifecycle related sub-commands
  msg        Various msg related sub-commands
  multicast  Various multicast related sub-commands
  node       Various node related sub-commands
  param      Various param related sub-commands
  pkg        Various package related sub-commands
  run        Run a package specific executable
  security   Various security related sub-commands
  service    Various service related sub-commands
  srv        Various srv related sub-commands
  test       Run a ROS2 launch test
  topic      Various topic related sub-commands

  Call `ros2 <command> -h' for more detailed usage.
```

### [Running Executables](https://youtu.be/wcibIqiRb04?t=385)
This is done by using `ros2 run`. The help provided by this command is the following:

```bash
ade$ ros2 run --help
usage: ros2 run [-h] [--prefix PREFIX] package_name executable_name ...                                                                                                                                                                      
Run a package specific executable                                                                                                                                                                                                            
positional arguments:                                                                                                                                                                                                                        
  package_name     Name of the ROS package
  executable_name  Name of the executable
  argv             Pass arbitrary arguments to the executable

optional arguments:
  -h, --help       show this help message and exit
  --prefix PREFIX  Prefix command, which should go before the executable.
                   Command must be wrapped in quotes if it contains spaces
                   (e.g. --prefix 'gdb -ex run --args').
```

It requires a package name and the name of the executable to run. It can also take some prefixes and arguments. For the example that will be used in this lecture, the syntaxis will be:

```bash
ros2 run turtlesim turtlesim_node
```

After running it, a simple simulation with a turtle that draws the path that it has followed is launched. The `turtlesim` package provides other nodes such as `draw_square` or `turtle_teleop_key` that will make the turtle draw squares or make the user able to teleoperate it, respectively. When using some of the additional nodes, they will be running in another terminal session and both will be communicating through ROS topics.

### [Inspecting Nodes](https://youtu.be/wcibIqiRb04?t=565)
There can be the case that we do not know or not remember how a ROS-based system works, specially when they are not as simple as the one showed before. For these cases, the first thing to do is knowing which components (nodes) does the system have. In order to do so, `ros2 node` is to be used.

```bash
ade$ ros2 node --help
usage: ros2 node [-h]
                 Call `ros2 node <command> -h` for more detailed usage. ...

Various node related sub-commands

optional arguments:
  -h, --help            show this help message and exit
                                                      
Commands:
  info  Output information about a node
  list  Output a list of available nodes
                                                                                                                                                                                                                                             
  Call `ros2 node <command> -h` for more detailed usage.
```

As shown in the help of this command, it can either **list the current active nodes** (`ros2 node list`) or provide **information about one of them** in particular (ros2 node info <node_name>). In particular, the `info` option will show which topics are being published or subscribed by the node (and the message type) and the services and actions that it is using.


## [3.3. Bus Sniffing for Topic Examination](https://youtu.be/wcibIqiRb04?t=755)
- ros2 topic list
- ros2 topic echo
- ros2 topic hz
- ros2 topic info
- ros2 topic show
- ros2 topic pub

## 3.4. GUI Equivalents
RQT and rqt_graph

## 3.5. Parameters
- ros2 param list
- ros2 param get
- ros2 param set

## 3.6.Services
- ros2 service list
- ros2 service type
- ros2 service show
- ros2 service call

## 3.7. Actions
- ros2 action list
- ros2 action info
- ros2 action send_goal
- ros2 action show
- More complex calls

## 3.8. Logging Data: Securing the Bag
- What is a *bag*?
- ros2 bag record
- ros2 bag record selecting topics
- ros2 bag info
- ros2 bag play

## 3.9. Wrap Up and Homework

