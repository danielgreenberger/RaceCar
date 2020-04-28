# ROS wrapper for RaceCar





## Introduction

This document will provide a short overview of ROS and its basic functionality and tools, 
and how they are integrated into the RaceCar project. 

The use of ROS in optional. 

### What is ROS
ROS (Robot Operating System) is a set of tools and libraries which help software developers create robot applications.
Although ROS is not a real OS , it provides services which are typically provided by operating systems. 


> ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers. ROS is similar in some respects to 'robot frameworks,' such as Player, YARP, Orocos, CARMEN, Orca, MOOS, and Microsoft Robotics Studio.
from: http://wiki.ros.org/ROS/Introduction

### ROS integration with RaceCar

Using RaceCar with the ROS framework has many advantages, as described above. 

The goal is to provide an easy option to integrate ROS with the current RaceCar flows, 
such that:
1. The use will be as simplified as possible.
2. Using ROS will be optional and not obligatory - we could always choose between ROS and regular mode
3. Minimal change to the system in order to adapt it to ROS

-- The use will be as simplified as possible.--
For this, wrapper classes and interfaces were created for ROS basic functionality. 

-- Using ROS will be optional and not obligatory --
The ROS integration code and definition is controlled by a preprocessor compilation flag, 
which is only activated when compiling for ROS. 
In ROS mode, the RealSense camera has a formal supported driver, making RaceCars wrapper classes redundant. 
Therefore, all camera code and definition was placed under a compilation flag, which allows to disable it
when we are using RaceCar with ROS, or for any other purpose. 

-- Minimal change to the system in order to adapt it to ROS --

The best way to adapt RaceCar to ROS would be to make it moduler, by separating all RaceCar components (RealSense, Bitcraze, Chaos, etc.)
into different execution nodes. 
Alas, doing it this way will make it harder to maintain the project code for both ROS and non-ROS enviroments, and
will complicate deploying the RaceCar in ROS. 

Therefore, RaceCar will remain as one process, and will not be a ROS node by itself. 
However, RaceCar will have objects which will register as ROS nodes (i.e publisher, see below). 



### Motivation

The integration with ROS has a few great advantages:
###### modularity 
ROS views the system in an Object oriented programming way. 
This is very helpful since many times the different components of the robot (which can be many) 
are loosely related to each other. This separation can be easier as a design and for debugging, 
as well as the option to use third-party drivers for some components (i.e RealSense)

###### infrastructure 
ROS provides easy ways to communicate between different components of the system, including a built-in Publisher-Subscriber model.  

###### Complementary tools  
ROS comes with many built-in and other usefull tools which can be used on the platform, 
including:

- Rviz 				     - A 3D visualization tool for ROS.
- Rosbag			     - A tools allowing to collect information published to ROS nodes and play them back later.
- TF                     - lets the user keep track of multiple coordinate frames over time, by maintaining the relationship between different coordinate 
						   frames and allows an easy transformation of points, vectors, etc. between coordinate frames at any desired point in time.
- Google Cartographer    - A tool providing SLAM
- Intel RealSense driver - Provides a wrapper for all RealSense funtionality and allows an easy use of the realsense device.


## ROS basic concepts

### Nodes

Nodes are the basic execution entities in ROS, representing the equivalent of a "thread" or "process" in traditional OS. 

For example a node can represent: a camera, motor, etc.. (this idea supports an OOP view of the system which is often useful)

Nodes communicate with each other using ROS facilities. 

TODO - explain clearly the connection between nodes, topics, and related infrastructure (service, publisher, listener, etc.)
TODO - (maybe add a table with comparision of ROS concepot to Linux, like process=Node etc)

### Messages and Topics

ROS nodes can communicate with each other by sending and receiving messages. 
TODO - continue

### Packages and Workspaces

#### overview

TODO - what is a package and whats the difference from node?



#### Creating workspaces with catkin

#### Sourcing the workspace
TODO - explain why we need to source the workspace

## ROS basic tools

### roscd

### rospack 
### rqt_launch 
This tools allows easy running of multiple nodes together. 
In our project it will be used for running RaceCar, RealSense driver and google cartographer. 

For more information: http://wiki.ros.org/rqt_launch

### rqt_console

ROS has a dedicated topic called "rosout" which is used as "printf" by nodes.  
This is a graphical tool allowing to view log prints of nodes in the ROS system. 

For more information: http://wiki.ros.org/rqt_console


## ROS integration with RaceCar
### ROS installation on Jeston
### ROS compilation enviroment
In order to avoid code duplication, the ROS integration will be embedded in the RaceCar project under special 
compilation flag. 

- A preprocessor macro for all ROS-related code and definitions

The compilation for ROS is done using catkin which is a modified version of cmake.

TODO insert a guide on how to create catkin workspace with cmake, step-by-step:
(maybe we can use catkin_create_pkg for some of the steps)
1. Create catkin workspace
2. Copy files
3. Edit cmake
4. Add other nodes (such as RealSense wrapper)
5. run catkin_make








## RaceCar RosIntegration API

### Ros publisher

This is a template class which serves as a simple wrapper for ROS topic publishing functionality. 



## Sensing HW used
## Examples
###### Code
###### ROS launch of all components
###### ROS compilation (create catkin ws, create makefile, compile)
###### Some useful scripts

## Bibliography
This section provides sources for more in-depth understanding of ROS
###### ROS reference links
http://wiki.ros.org/roscpp/Overview - ROS c++ library -- documentation

###### Catkin

http://wiki.ros.org/catkin/Tutorials - Catkin tutorial
http://wiki.ros.org/catkin/Tutorials/using_a_workspace - Building a package with catkin_make
http://wiki.ros.org/catkin/CMakeLists.txt   -  How to create a Cmake file for catkin


## TODO - delete after finishing
- [] Ros launch
- [] Add implementation-related documentation here or in the c file (for example publisher/listener)
- [] Add info about ROS launch
- [] Change RaceCar log prints to a wrapper function/macro and direct all log prints to ROS_LOG
- [] Add a small section with "Get started" which runs rqt_launch and rqt_console and chooses different launch files
- [] Add information about TF