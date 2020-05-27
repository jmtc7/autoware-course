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


## [2.3. Nodes, Publishers, and Subscribers](https://youtu.be/FTA4Ia2vLS8?t=1485)
The core of ROS is the pub/sub bus, which is called *topic*. Each *topic* has a message type that is published on it. These **message types** are defined in yaml files that specifies what do they contain. An example of a ROS message type is `geometry_msgs/Twist`, which is a message to publish velocities. In its definition file, a *Twist* message is defined as two vectors with 3 float elements each. One vector is for the linear velocity and the other for the angular one, being their elements the X, Y and Z elements of those magnitues. There are a lot of pre-defined robotics-oriented message types, but ROS makes it possible to define new personalized ones.
an DL
ROS nodes can publish messages to topics (this would be known as a **publisher node**) or subscribe to topics to read the messages that are being published into them (**subscriber node**). The same node can be subscriber and publisher of one or more topics.

ROS provides many **tools** for managing topics that can be used with the `ros2 topic` command. Some of the possibilities are list, *echo* them (print what is being published on them), *remap* (renaming the topic), or *bag* them (store what is being published on them in a file to replay them later on).

ROS **nodes** are basically programs that run concurrently on ROS. Again, the `ros2 node` command provides a lot of node-related **tools**. It is possible to execute them independently using the `ros run` command, but it is also possible to configure custom grupal executions using ROS *launch files*.

### [Building and Running a ROS Node](https://youtu.be/FTA4Ia2vLS8?t=1620)
The first things to do after opening a new terminal are **entering the ADE environment** and **sourcing the ROS installation and the workspace** where the node that is going to be used is located. It is a good practice to use different terminals for building and executing. The package containing the node that is being executed can be built using the procedure explained in the *Colcon Nomenclature* subsection of this document. The sourcing of the examples package is done by executing `source install/setup.bash` from the root of the workspace.

Once the code is built, it is possible to **run the node** (executable file generated from the building of the source code). To execute a node (executable) called `publisher_lambda` from the ROS package `examples_rclcpp_minimal_publisher`, the syntax would be `ros2 run <package_name> <node_name>`, so the command to be executed (and its output) would be:


```bash
ade$ ros2 run examples_rclcpp_minimal_publisher publisher_lambda
[INFO] [minimal_publisher]: Publishing: 'Hello, world! 0'
[INFO] [minimal_publisher]: Publishing: 'Hello, world! 1'
[INFO] [minimal_publisher]: Publishing: 'Hello, world! 2'
[INFO] [minimal_publisher]: Publishing: 'Hello, world! 3'
```

As an additional note, *RCLCPP* stands for *ROS Common Library C++*. 

What this node is doing is publishing a string at 2 Hz in a topic called `/topic`. A list of the active topics can be seen by using `ros2 topic list` in another terminal. The messages that are being published in a topic can be monitored by using `ros2 topic echo <name_of_the_topic>`, and the frequency at which messages are being published can be estimated with `ros2 topic hz <name_of_the_topic>`.

### [Publisher's Code](https://youtu.be/FTA4Ia2vLS8?t=1915)
The code in `ros2_example_ws/src/examples/rclcpp/minimal_publisher/member_function.cpp` (from the previously executed node) will be used as an example. Regarding the **dependencies**, `rclcpp/rclcpp.hpp` is the C++ library for ROS 2. On the other hand, `std_msgs/msg/string.hpp` is included in order to be able to use the `string` ROS message type, which is inside the `std_msgs` package.

Next, the `MinimalPublisher` **class** is defined, which inherits from `rclcpp::Node`, which means that the defined class will be a ROS node written in C++. Inside this class, the following elements are contained:

- **Constructor**: It will receive a name and member variable (`counter_`), that will be the times that the node has published a message. The publisher is created to publish message of the type `std_msgs::msg::String` in the topic `/topic`, keeping a queue of up to 10 messages. Then, a 500 ms timer is created and binded to a callback (`timer_callback`).
- **Timer's callback**: Whenever the timer completes its count, this function will be executed. It starts creating an empty `String` ROS message. Next, it stores `Hello, world!` followed by the content of the `count_` variable (adding one to it on the process). It prints what is going to be published using the ROS INFO and, finally, publishes it using the publisher created in the class constructor.
- **Private member variables**: The `timer_`, `publisher_` and `count_` variables used in the above mentioned functions.

The last part of the node is the `main()` function, which initializes the ROS Common Library (RCL) with `rclcpp::init()`, runs the `MinimialPublisher` node (spinning it) and cleans and closes everything whenever the spinning is interrupted.

### [Subscriber's Code](https://youtu.be/FTA4Ia2vLS8?t=2440)
Their usage is to read from a data stream (messages published in a topic) and, most of the time, process it and do something in consequence of the result. For example, there is a node acting as the driver of an encoder publishing when something each time a wheen turns one degree, a subscriber may subscribe to this information and process it to transform it from clicks to RPMs and publish it into another topic, which may be used by another subscriber to know if the current velocity is too high or too slow to variate the control signals sent to the wheels.

The example code here will be `ros2_example_ws/src/examples/rclcpp/minimal_subscriber/member_function.cpp`. Again, the RCLCPP and String **header files** are included, since the ROS C++ library and the ROS String message will be used aswell. Again, same as before, a **class** is declared and it inherits from the RCLCPP's `Node` class, but this time it is called `MinimalSubscriber`. Its components are:

- **Constructor**: Creates a subscription to a topic named `topic` expecting to read messages of type `std_msgs::msg::String`. This subscription will be binded to a callback function named `topic_callback()`, which will be executed each time a new message is received.
- **Subscription's callback**: It will receive as its only argument a shared pointer to the new message that has been received. The callback will only send through ROS INFO that the message was received and the content of the message.

For the `main()` function, again, the C++ ROS Common Library is initialized (with `init()`), the node is created and spinned until its execution is cancelled and, when it occurs, the program will clean everything and exit (`shutdown()`).

By having both the publisher and the subscriber nodes running at the same time, the subscriber will be able to receive the published messages thanks to ROS 2. Any of these nodes' executions can be killed using `Ctrl + C` as with any normal CLI program. 


## [2.4. Services](https://youtu.be/FTA4Ia2vLS8?t=2885)
### [Overview and basic example](https://youtu.be/FTA4Ia2vLS8?t=2885)
Nodes, topics, and messages are the basics of ROS and many robots use only those elements to work. These elements are great to move a lot of data and make quick processing of it. However, there are occasions in which the robots must respond to data with simple behaviours, which is done with ***ROS Services***. They are similar to calling functions but the function is implemented in another node. They are tasks performed synchronously, i.e. they are performed while the thing that is using them waits (they can be called both from nodes or from the command line).

The services are defined inside a folder named `srv` inside the package folder and they specify the inputs and the outputs of the service in a `.srv` file, which uses YAML syntax. A ***service server*** will be also necessary, which will be a node implementing what does the service does with its inputs and how it computes its outputs. The node using the service will be named a *service client*. A **sample service file** can be found at `/opt/ros/dashing/share/example_interfaces/srv/AddTwoInts.srv`, which defines a service whose purpose will be tu return the sum of two integers. The content of the file is the following:

```yaml
int64 a # An input of type int64 called 'a'
int64 b # An input of type int64 called 'b'
---
int64 sum # An output of type int64 called 'sum'
```

A more in-depth tutorial on services and how to create and use them is in the following [ROS Tutorial](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29).

### [Defining a Service Server](https://youtu.be/FTA4Ia2vLS8?t=3105)
Using the `AddTwoInts` service mentioned before will be equivalent to using (in pseudocode) something like this: `int64 sum = AddTwoInts(int64 a, int64 b)`.

A sample implementation of a *service server* (that defines what will happen when the service is called) can be found in `/home/jmtc7/ros2_example_ws/src/examples/rclcpp/minimal_service/main.cpp`. `Colcon` will generate the `add_two_ints.hpp` **header file** , while the other is the C++ ROS Common Library. The `using [...]` instruction is there in order to improve code clarity.

The `handle_service()` function is what is going to be executed every time the service is called, which will take as arguments shared pointers to the request header (metadata of the request), to the request itself (input data, i.e. `a` and `b`) and to the response (output data to be written, i.e. `sum`). The function itself only sends through ROS INFO the received request and assigns the output with the sum of the inputs.

Once having the service implemented, a `main()` is needed to make it available for the rest of the ROS environment. This `main()` initializes ROS, creates a node named `minimal_service` (which hosts the service server), creates the server service (named `add_two_ints`) and binds it to `handle_service()` and, as it was done with the publishers and subscribers, this will just spin until the failure or other execution interruption, moment in which everything will be cleaned and shutted down.

### [Building and Running Services](https://youtu.be/FTA4Ia2vLS8?t=3315)
In order to run a service, after building with `colcon` as already done before, the first thing will be to run the node hosting the service server, for which the (already used) syntax will be `ros2 run <package> <service server>`:

```bash
ade$ ros2 run examples_rclpp_minimal_service service_main
```

Now it is possible to send requests to (and receive responses from) this service, both from the command line or from a node. For **calling a service using the command line**, the syntax is `ros2 service call <service name> <service type> <data to input>`, resulting in something like the following:

```bash
ade$ ros2 service call /add_two_ints example_interfaces/AddTwoInts "{a: 1, b: 1}"
```

An useful tool is `ros2 service list`, which can be executed to list all the existing services.

In order to **call a service from code**, a *service client* needs to be implemented. This is, a node that uses a service. An example of this is at `/home/jmtc7/ros2_example_ws/src/examples/rclcpp/minimal_client/main.cpp`. The already used header files are seen, as well as the `using` sentence. Inside the `main()`, after initializing ROS and creating a node, a client to the `add_two_ints` service is created. Notice that the ROS way of naming things is using camel case for the types (`AddTwoInts`) and underscores and lower case letters for the node names (`add_two_ints`). It checks if the service is available, throwing an error if it is not. Then, it will create a request, assign values to it and use the client to send it to the service, assigning the returned value to the result. Next, it will check if the return code is not `SUCCESS`, in which case an error will be thrown and the program will end returning a failure code (`1`). If it is not the case, the result will be betted and log it into ROS INFO.

After building and having the server running, the client can be ran by executing:

```bash
ade$ ros2 run examples_rclpp_minimal_client client_main
```

Both the client and the server should show their logs through ROS info.


## [2.5. Actions](https://youtu.be/FTA4Ia2vLS8?t=3760)
- Action Overview
- Action file review
- Basic action review
- Running/calling an action
- Action client review
- Running an action server with a client


