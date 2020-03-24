# ROS wrapper for RaceCar





## Introduction

ROS (Robot Operating System) is a set of tools and libraries which behave as pesudo-OS which help software developers create robot applications.

Although ROS is not a real OS , it provides services which are typically provided by operating systems. 

ROS (Robot Operating System) provides libraries and tools to help software developers create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. ROS is licensed under an open source, BSD license.

TODO add explanation about ROS (its a meta-OS, open source, designed for Robot builders) 


This document descripbes the integration of the RaceCar platform with with ROS.  
(describes basic concepts and tools of ROS along RaceCar wrapper API with ROS)

## Motivation

The integration with ROS has a few great advantages:
###### modularity 
ROS views the system in an Object oriented programming way. 
This is very helpful since many times the different components of the robot (there can be many) 
are loosely related to each other. This separation can be easier as a design and for debugging, 
as well as the option to use third-party drivers for some components (i.e RealSense)

###### infrastructure 
ROS provides communication between nodes

###### Third party support  
ROS comes with many built-in and other usefull tools which can be used on the platform
For example RealSense has a ROS driver (better than re-implementing)

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

The compilation for ROS is done using catkin which is a modifies version of cmake (TODO re-write it more clearly)

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