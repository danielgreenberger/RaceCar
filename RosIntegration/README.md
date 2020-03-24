# ROS wrapper for RaceCar





## Introduction

RaceCar is a robust robotic car platform which can be used for a wide variety of applications. 

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

### Topics

### Packages and Workspaces

#### overview

TODO - what is a package and whats the difference from node?



#### Creating workspaces with catkin

#### Sourcing the workspace
TODO - explain why we need to source the workspace

## ROS basic tools

### roscd
### rospack 


## ROS integration with RaceCar
### ROS installation on Jeston
### ROS compilation enviroment
- A preprocessor macro for all ROS-related code and definitions

The compilation for ROS is done using catkin which is a modifies version of cmake.

TODO insert a guide on how to create catkin workspace with cmake, step-by-step:
(maybe we can use catkin_create_pkg for some of the steps)
1. Create catkin workspace
2. Copy files
3. Edit cmake
4. Add other nodes (such as RealSense wrapper)
5. run catkin_make








## RaceCar RosIntegration API

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


## TODO - delete after finishing
- [] Ros launch
- [] Add implementation-related documentation here or in the c file (for example publisher/listener)
- [] Add info about ROS launch