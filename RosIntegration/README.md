# ROS wrapper for RaceCar



This document describes the integration the RaceCar project with ROS, and consists of 2 parts. 

The first part provides a brief overview of the ROS system, its basic functionality and its tools.

The second part is more practical, and will include a guide to [building with ROS](#creating-catkin-workspace-and-package) as well as a description the RaceCar-ROS integration, along with the the [wrapper API that were created.](#RaceCar-RosIntegration-API). 

Please note that the use of ROS in optional, and may not be suitable for all use-cases. 


## Introduction 



### What is ROS

ROS (Robot Operating System) is a set of tools and libraries which help software developers create robot applications.
Although ROS is not a real OS , it provides services which are typically provided by operating systems. 

from ROS official website:

> ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers. ROS is similar in some respects to 'robot frameworks,' such as Player, YARP, Orocos, CARMEN, Orca, MOOS, and Microsoft Robotics Studio.

from: http://wiki.ros.org/ROS/Introduction



### Motivation

The integration with ROS has a few great advantages:

###### Modularity 

ROS has a very Object Oriented narrative. 
Many differents Robot components such as Motor controller or Camera can be represented independantly in the system, 
apart from other components. 

This is very helpful since many times the different components of the robot (which can be many) 
are loosely related to each other. 

Such modular approach can also be easier to design and for debugging.  
It also gives us the option to integrate third-party libraries for our components, such as the [RealSense ROS driver](https://github.com/IntelRealSense/realsense-ros).


###### infrastructure 

ROS provides a built-in infrastructure which answers the needs of many robot system, such as it's Publish-Subscriber model of [messages and topics](#Messages-and-Topics).  


###### Complementary tools  
ROS has a growing and thriving community which develops many useful tools. 
Some tools come shipped with ROS, but many other open-source libraries can be downloaded and used. 

This document briefly discusses some of the important libraries, but many others can be found at the 
[ROS official website](https://index.ros.org/packages/)





## ROS basic concepts



### Nodes

Nodes are the basic execution entities in ROS, representing the equivalent of a "thread" or "process" in traditional OS. 

As an example, a node can represent a camera or motor controller module. 

Nodes communicate with each other using ROS facilities. 

> A node is a process that performs computation. Nodes are combined together into a graph and communicate with one another using streaming topics, RPC services, and the Parameter Server. These nodes are meant to operate at a fine-grained scale; a robot control system will usually comprise many nodes. For example, one node controls a laser range-finder, one Node controls the robot's wheel motors, one node performs localization, one node performs path planning, one node provides a graphical view of the system, and so on.

from: http://wiki.ros.org/Nodes



### Messages and Topics

ROS nodes can communicate with each other by sending and receiving messages of a specific *topic*

> Topics are named buses over which nodes exchange messages. Topics have anonymous publish/subscribe semantics, which decouples the production of information from its consumption. In general, nodes are not aware of who they are communicating with. Instead, nodes that are interested in data subscribe to the relevant topic; nodes that generate data publish to the relevant topic. There can be multiple publishers and subscribers to a topic.

from: http://wiki.ros.org/Topics


Many messages of different types can be sent and received using ROS topics. 

Many messages types are available by the **common_msgs** package, which comes shipped with ROS. 
Other types can be found in third-party libraries or defined by the user. 

The messages in **common_msgs** are often used as the standard messages by many applications (i.e Google Cartographer), so it's better
to try using them before re-inventing the wheel. 


### Packages and Workspaces

> Software in ROS is organized in packages. A package might contain ROS nodes, a ROS-independent library, a dataset, configuration files, a third-party piece of software, or anything else that logically constitutes a useful module. The goal of these packages it to provide this useful functionality in an easy-to-consume manner so that software can be easily reused.

from: http://wiki.ros.org/Packages


A workspace is a collection of packages, organized using a built-in tool called **catkin**.

To begin creating code for ROS using catkin, it is useful to read the [following tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

 




## ROS basic tools

ROS comes with many built-in and other useful tools which can be used on the platform, 
including:



### Command line tools

This group includes tools which are used for easily navigating the ROS filesystem, running [nodes](#Nodes) 
and getting useful debug information. 

A (very) partial list includes:

- **roscd** 				 - A tool offering an alternative to the 'cd' command in the terminal, allowing to switch directories by package name instead of absolute/relative path.

- **rosrun** 				 - Similarly to roscd, allows to **run** nodes from packages easily.

- **rospack** 				 - A tool for retrieving information about ROS packages available on the filesystem. 

- **rostopic** 				 - Display information about active ROS topics (i.e publish-frequency, listen to a topic, etc.)

- **Rosbag**			     - A tool allowing to collect information published to ROS nodes and play them back later.

**Note:** In order to use some of the tools you must source your catkin workspace. Check ROS documentation for more details. 



### Graphic tools

**rqt** is a convenience tool which group many useful GUI tools for ROS. 

Some of the most useful tools include:

##### rqt_launch 

This tools allows easy running of multiple nodes together. 
In our project it will be used for running RaceCar, RealSense driver and google cartographer. 

For more information: http://wiki.ros.org/rqt_launch

##### rqt_console

ROS has a dedicated topic called "rosout" which is used as "printf" by nodes.  
This is a graphical tool allowing to view log prints of nodes in the ROS system. 

For more information: http://wiki.ros.org/rqt_console

##### rviz

A 3D visualization tool for ROS.
rviz can be used to visualize a lot of useful information which is available from active topics. 

Some examples include plotting a robot path in different coordinate systems, maps, 
various camera data and more. 

The link below contains detailed information about rviz, including tutorials for beginners.

For more information: http://wiki.ros.org/rviz


### Other tools and libraries


- **TF**                     - lets the user keep track of multiple coordinate frames over time, by maintaining the relationship between different coordinate 
						       frames and allows an easy transformation of points, vectors, etc. between coordinate frames at any desired point in time.
- **Google Cartographer**    - A tool providing Simultaneous localization and mapping (SLAM)
- **Intel RealSense driver** - Provides a wrapper for all RealSense funtionality and allows an easy use of the realsense device.









## ROS and the RaceCar project


### ROS integration with RaceCar

Using RaceCar with the ROS framework has many advantages, as described above. 

The goal was to provide an easy option to integrate ROS with the current RaceCar flows, which will be:

1. Simple to use                  -  the use will be as simplified as possible.
2. Simple to deploy               - we could always switch between ROS and another enviroment.
3. Minimal change to RaceCar code - This is in order to avoid code duplication and increase portability. 

#### Simple to use
For this, wrapper classes and interfaces were created for ROS basic functionality. 

#### Simple to deploy
The ROS integration code and definition is controlled by a preprocessor compilation flag, 
which is only activated when compiling for ROS. 


#### Minimal change to RaceCar code

The best way to adapt RaceCar to ROS would be to break it into differnt components (RealSense, Bitcraze, Chaos, etc.)
and define each as a node.
However, this approach has a drawback, as a modular approach will make it harder to maintain the project code for both ROS and non-ROS enviroments, as well
as complicate the deployment of RaceCar under ROS. 

As a compromise, we will keep the RaceCar process intact and use ROS through wrapper classes. This is discussed by the next section. 




### High Level Design overview

The main idea was to create an easy-to-use interface of ROS, while still allowing portability of the RaceCar code to other platform (specifically, to just running RaceCar directly from the OS). 

In order to archive this balance, the following design decisions were made:

1. The use of ROS funtionality is done through wrapper classes which were developed for this project (currently only a publisher class is available). 
   Those classes expose a very simple interface, keeping the ROS-specific details to the internal implementation. 
   This way it will be easier in the future to port the code to other platforms. 
2. The RaceCar process itself is not a ROS node. To be more exact, although it's technically run as a node, it doesn't register as a node is the ROS system and
   therefore doesn't have a name. 
   The reason is again portability, as we aimed to mask out ROS specific details from the RaceCar interface. 
   Instead, every instance created from the wrapper classes (i.e RosIntegration::Publisher) registers as a names node, 
   and all interaction with ROS is done through those wrapper classes. 


Each instance of the wrapper classes is performed on a different thread, and the exchange of information (with RaceCar main thread) is done through
FIFO memory buffers (For more information see documentation in the spefific files). 

This approach was decided for a few reasons:

1. Nvidia Jetson supports multi-threaded excecution, and we wanted to use this functionality for better performance. 

2. Separate threads allow more fine-grained control of critical-timing flows.  
   This is because the overhead on the main thread only includes the data-copy into the buffer, therefore having a minimal impact on the flow latency. 


### RaceCar compilation flavors

In order to avoid code duplication, some of the functionality was put under #ifdef compilation flags.

##### ROS_COMPILATION

All ROS-related code and definition (including the wrapper classes) will be put under this flag.  
When [creating the cmake for the ROS build](#step-3-makefile-configuration) we add the ROS_COMPILATION flag to include the
RosIntegration libraries. 

For other platform, we don't define this flag and therefore all the ROS-related code is redacted by the compiler. 

##### NO_CAMERA

If set, all the realsense-related code of RaceCar is redacted by the compiler. 

This flag is needed when we want to use the [official realsense driver for ROS](https://github.com/IntelRealSense/realsense-ros)

When the offical driver is used, it is run as a separate node to RaceCar and manages the camera, 
and therefore we need to disable our RaceCar camera API. 
 
When not using ROS, or using ROS without the official driver, we will not define this flag and therefore will be able to use 
the RaceCar camera API in the usual way. 



## Getting started

### ROS installation
First, make sure that you have ROS installed on your system.
ROS installation guide can be found [here](http://wiki.ros.org/ROS/Installation).

The next step would be to create a workspace and package using catkin

### Creating catkin workspace and package

The build process under ROS is done using the **catkin** tool, which is based on a modified version of cmake.
The rest of this sections will explain how to work with catkin.

##### Step 1: Creating the workspace
First go to the path where you wish to place your workspace, which can be any path where you have read/write/execute persmissions.

Then create the workspace folder:
```
mkdir -p catkin_ws/src
```

If you want to use this workspace with various node tools, you will need to source it:
```
source catkin_ws/devel/setup.bash 
```

##### Step 2: Creating the racecar package
The next step would be to create a package for racecar inside the workspace we just created.

``` 
catkin_create_pkg racecar roscpp common_msgs tf 
```

catkin_create_pkg is a convenience script for creating a new package.

The first argument (racecar) is the name of the package, and the rest of the arguments are dependency packages - these are 
packages racecar depends on for compilation/execution. 

**roscpp**       -  This is the ROS c++ API library.

**common_msgs**  -  A package containing all standard ROS messages to be published to different topics

**tf**           -  A package responsible for keeping track of different coordinate systems.



##### Step 3: makefile configuration

Under catkin_ws/src/racecar you will find a CMakeLists.txt. 

Add the following lines under "build" section:


```
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## This is the preprocessor flag which define the ROS compilation flavor (if not defined, all ROS code is redacted by the preprocessor)
add_definitions(-DROS_COMPILATION)  


## This is because we use the intel RealSense wrapper for ROS
add_definitions(-DNO_CAMERA)  


## Add linker flags
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Chaos/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Chaos/RaceCar.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Chaos/RaceCar.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Chaos/Chaos_types.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/TcpClient/TcpClient.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/TcpClient/ITcpClient.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/motorAPI/Arduino.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/motorAPI/MotorController.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cameraAPI/RealSenseAPI.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/TcpServer/ITcpServer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/TcpServer/TcpServer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/jpeg/JpegCompressor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Serial/Serial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Serial/ISerial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bitcraze/bitcraze.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/RosIntegration/ros_lib.cpp

    )


set(INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/TcpClient
    ${CMAKE_CURRENT_SOURCE_DIR}/src/TcpServer
    ${CMAKE_CURRENT_SOURCE_DIR}/src/motorAPI
    ${CMAKE_CURRENT_SOURCE_DIR}/src/arduino/bitcraze
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cameraAPI
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Serial
    ${CMAKE_CURRENT_SOURCE_DIR}/src/jpeg
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bitcraze
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Common/Coordinates
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Common/Utils
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Chaos
    ${CMAKE_CURRENT_SOURCE_DIR}/src/RosIntegration
    )


INCLUDE_DIRECTORIES(${INCLUDES})

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME}
    -lrealsense2
    -lturbojpeg
    -lz)

```


**Note:** The lines above correspond to the Cmake configurations for RaceCar, and need to be updated whenever the racecar 
cmake is changed (i.e new file added/deleted).

##### Step 4: Copy all source files

Copy all of racecar source files to catkin_ws/src/racecar/src

##### Step 5: Compile

The next step would be to compile the workspace. 
This is done by going to the workspace root firectory (catkin_ws in our example), 
and running 
``` 
catkin_make 
```

This will build all the packages in the workspace. 

catkin_make has some other options such as building a specific package or forcing a re-build. 
See documentation for more details. 

##### Step 6: Running

Last but not least, we would like to run the RaceCar node. 
If you sourced the workspace, just run:

``` 
rosrun racecar racecar 
```

*Note:* The sourcing of a workspace is only valid for the current shell terminal, since one can have many workspaces (for example, a workspace for the Google Cartographer). 
If you use a certain workspace frequently, it might me useful to add the source command to ```~/.bashrc```




## RaceCar RosIntegration API

### Ros publisher

This is a template class which serves as a simple wrapper for ROS topic publishing functionality. 

The publisher is intergated to the main, non-ROS-native program as a class member. 

The publisher object is comprised of 2 main components:

1. A FIFO memory buffer.
	
2. A thread which registers on the ROS system and publishes the FIFO content to a given ROS topic.  
	

The implementation using a buffer is done is order to minimize the latency on the RaceCar 
main thread. 

The drawback is an increased latency in publishing a single message to ROS, since it has to go through the FIFO. 

### Ros Odometer publisher

This class is a derived type of the general ROS publisher which is intended for publishing odometer data. 

In addition to being a child class of ros_publisher, this class also utilizes the TF 
library to define and maintain different coordinate systems for the odometer, robot baselink
and a static world frame-of-reference (i.e Lab starting point). 




## Bibliography

This section provides sources for more in-depth understanding of ROS

###### ROS reference links


http://wiki.ros.org/roscpp/Overview - ROS c++ library -- documentation

###### Catkin

http://wiki.ros.org/catkin/Tutorials - Catkin tutorial

http://wiki.ros.org/catkin/Tutorials/using_a_workspace - Building a package with catkin_make

http://wiki.ros.org/catkin/CMakeLists.txt   -  How to create a Cmake file for catkin




## TODO - delete after finishing

- [] Add info about ROS launch

- [] Change RaceCar log prints to a wrapper function/macro and direct all log prints to ROS_LOG

- [] Add a small section with "Get started" which runs rqt_launch and rqt_console and chooses different launch files

