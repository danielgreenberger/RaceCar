# RaceCar




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

#### Nodes

Nodes are the basic execution entities in ROS, representing the equivalent of a "thread" or "process" in traditional OS. 

For example a node can represent: a camera, motor, etc.. (this idea supports an OOP view of the system which is often useful)

Nodes communicate with each other using ROS facilities. 

TODO - explain clearly the connection between nodes, topics, and related infrastructure (service, publisher, listener, etc.)
TODO - (maybe add a table with comparision of ROS concepot to Linux, like process=Node etc)

#### Topics





## RaceCar RosIntegration API
## Software environment
## Sensing HW used
## Examples
###### Code
###### ROS launch of all components
###### ROS compilation (create catkin ws, create makefile, compile)
###### Some useful scripts

## Bibliography
This section provides sources for more in-depth understanding of ROS
###### ROS reference links
