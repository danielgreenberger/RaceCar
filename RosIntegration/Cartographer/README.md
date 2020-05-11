# Introduction

This deocument shows an example for using the RosIntegration of RaceCar for SLAM applications. 
Specificaly, we show how to run the [Google cartographer](https://google-cartographer.readthedocs.io/en/latest/) and how to tune 
RaceCar's sensing devices to the inputs required by the Cartographer. 

The [first section](Cartographer-input) will provide a short overview of the input expected by the Cartographer, as well as the available output from the sensing devices of RaceCar 

The [second section](TODO insert link) is a practical guide for running the Cartographer under RaceCar.
If you are new to ROS, it is also recommended to read the [RosIntegration documentation] to get up-to-speed with ROS basic functionality and RaceCar wrapper APIs.

The [third and last](TODO insert link) section describes the iterative process of tuning the input from RaceCar sensing devices in order to construct a coherent map using Cartographer. 



# Cartographer input and sensing output.
## Cartographer input

A comprehensive guide on the required input can be found [here](https://google-cartographer-ros.readthedocs.io/en/latest/ros_api.html), under **Subscribed Topics**. 

The Cartographer can perform 2D or 3D mapping. 
This document focuses on 2D mapping as the output is easier to verify against a ground-truth.
However, this guide can be easily used for 3D mapping with minor modification, involving a small parameter change. 

### Range data
The Cartographer requires some range data as an input, such as PointCloud or Laser scan. Exactly one kind of range data can be supplied. 


### IMU (gyroscope and linear acceleration)

The cartographer may be supplied with IMU data, although it's optional for 2D mapping.
Nevertheless, we found out that IMU is needed for constructing a good map, especially gyroscope (for keeping track of the robot orientation during the SLAM process).


### Odometer
The Cartographer can also be provided with odometer data, which is the distance travelled from a fixed point.
Odometer data is not *required* by the cartographer, but similarly to IMU - it is highly needed for a good mapping.
 

The sensing devices used by the Cartographer:

### Intel RealSense depth Camera D435i
	
	This is a camera able to provide depth-image as well as IMU data (gyroscope and linear acceleration). 
	The depth-image can be used to produce PointCloud (using the RealSense API) or Laser scan (using the [depthimage-to-laserscan](https://wiki.ros.org/depthimage_to_laserscan) ROS package).
	

### Bitcraze Flow Breakout

	This device can be used to provide Odometry data.
	The output of the device includes the distance travelled in the X-Y direction as well as the distance from the floor. 
	The Bitcraze output needs to be sampled constantly and converted to the X-Y distance in meters. 
	Then, the new orientation of the robot needs to be determined, and the distance will need to be converted 
	to the lab fixed frame. 
	
	For more information on how to process the output, you can view the RaceCar implementation at [odometer_calculations.h](https://github.com/danielgreenberger/RaceCar/blob/master/Common/Coordinates/odometer_calculations.h) and [racecar_coordinates.h](https://github.com/danielgreenberger/RaceCar/blob/master/Common/Coordinates/racecar_coordinates.h)
	
	**Note**: As of writing this document, we couldn't get a reliable reading from the bitcraze device. However, we developed and tested the entire software infrastructure for processing and publishing Odometer data, based on an accurate Bitcraze reading. More info can be found at the files mentioned above. 
	As for the implementation, we tried avoiding using ROS-specific tools to make it portable to other platforms. 
	
	
	
	


# Getting started

This section will describe how to run Google Cartographer under the Racecar environment using ROS. 



Open 3 different terminals, which will be used for the **RaceCar**, **RealSense** and **Cartographer** nodes.

## Step 1: Sourcing the workspaces



First make sure you have sourced all the workspaces (one workspace needs to be sourced for each terminal):

### Cartographer  (terminal 1)

```
source /home/nvidia/catkin_ws_cartog_2020/install_isolated/setup.bash
```

Alternatively, if you want to source it by default you can add it to the bash environment (not recommended, since the default ws is currently the RealSense workspace):

```
echo "source /home/nvidia/catkin_ws_cartog_2020/install_isolated/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


**Note:** 
If you haven't yet installed google-cartographer, see the [official guide](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation) for more details. 
We recommend not using the ninja build as it may compicate the compilation process under the Jetson linux kernel. 
During the installation we found some packages to be missing:
- glog (Google log)
- ceres-solver

If they are not installed automatically, just install them using apt-get.



### RealSense  (terminal 2) 

**This step is relevant only if you chose to use the official ReslSense ROS wrapper**

The Realsense library should be sourced by default. 
If this is not the case:

```
echo "source /home/nvidia/catkin_ws_realsense_2020/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


### RaceCar  (terminal 3) 

```
source /home/nvidia/daniel_greenberger/final/catkin_ws/devel/setup.bash
```






## Step 2 : Running the RealSense camera node

**This step is relevant only if you chose to use the official ReslSense ROS wrapper**

run in the terminal:

```
roslaunch realsense2_camera cartographer.launch
```

**cartographer.launch** is a modified launch file located at ``` /home/nvidia/catkin_ws_realsense_2020/src/realsense-ros/launch ``` and is based on demo_pointcloud.launch. 

We chose to base on this specific launch file as it had the PointCloud option already included.

The modifications performed to demo_pointcloud.launch:
1. Add the IMU topic, as this ia required by the Cartographer for 3D mapping. 
2. Perform [remapping](http://wiki.ros.org/roslaunch/XML/remap) of the PointCloud and IMU topics, so they will match with the topics the Cartographer listens to. 

**Note:** The remmaping of topics can also be done at the Cartographer lua script. In such case, we would remap in the reverse direction (from the Cartograoher input names to the RealSense names). To see an example of such ramapping look at the other cartograoher launch files.  



## Step 3 : Running the RaceCar node

In **terminal 3**:

```
rosrun racecar racecar
```



## Step 4 : Running the Google Cartographer


To run the cartographer, simply run:

```
roslaunch cartographer_ros racecar.launch
```

**racecar.launch** is a launch file we created which runs the cartographer with specific configurations, 
which are found in the **racecar_2d.lua**

To understand how to config the launch and lua files, please visit the [Cartographer documentation](https://google-cartographer-ros.readthedocs.io/en/latest/your_bag.html).

We have used the online running option, so our launch file was based on **my_robot.launch**


**To collect the mapping results** you can open rviz and subscribe to the map topic.



# Algorithm tune-in

## Running attempt #1: PointCloud2, no Odometer

The first mapping is described below.
We chose to first get a good mapping in 2D before trying the 3D mapping, 
as 2D maps are somewhat easier to imterpret and verify.

**Depth image type:**  PointCloud, as supplied by the RealSense ROS node. 

**Odometry used:** No odometry data (Bitcraze outputs garbage values.)

**Mapping type (2D/3D):** 2D


**More info:** 

We walked the robot around the lab (carried by hand) in a path following around the walls. 
The path included the main lab area as well as the small corridor, excluding the VISTA lab and other rooms such as the kitchen. 

The robot was held parrallel to the ground in a height of ~1.5 meters.


![running_attempt_1_a](images/rviz___first_run_attempt_pointcloud_2d_walking_the_lab.png)
![running_attempt_1_b](images/rviz___first_run_attempt_pointcloud_2d_walking_the_lab2.png)

**Results**

Looking at the output figures (collected using rviz),
we can see that the initial results is not accurate at all. 
However, looking the zoom-in image (second figure) we may be able to see the corridor. 





## Running attempt #2: Laser data, no Odometer, static positioning of the robot


The conversion to Laser data was made using the [depthimage-to-laserscan](https://wiki.ros.org/depthimage_to_laserscan) package.

Install:
```
sudo apt-get install ros-melodic-depthimage-to-laserscan
```

Then, add it to the end of the RealSense launch file:
```
	<node name="depthimage_to_laserscan" pkg="depthimage_to_laserscan" type="depthimage_to_laserscan" > 
  	<remap from="image" to="/camera/depth/image_rect_raw"/>
	</node>
```


**Depth image type:**  Laser scan, produced by RealSense depth image and converted using depthimage_to_laserscan.

**Odometry used:** 

No odometry data (Bitcraze outputs garbage values). 

This seems to be the major cause of failure, as will be described.



**Mapping type (2D/3D):** 2D


**More info:** 

In order to neglect the (lack of) odometer - in this experiment we mounted the robot on a rotating chair at
the entrance to the corridor. 
The robot was moved slightly by making small rotations of the chair. 
The position of the robot as well as the chair are presented below


**Results**

![running_attempt_2_a](images/rviz__second_attempt_corridor_vs_map.jpg)


As we can see, using static positioning, we can see how the Cartographer uses the Laser data to construct a map of the corridor. 

On the right-hand-side of the map, we can see a blurred-image of the recycling bin.

Looking at the results, we can deduce the following:

1. Although optional, an accurate Odometer is an important part of the SLAM process and should be supplied to the Cartograoher. 
2. In cases where an Odometer is not relevant (i.e the robot is at the same position), we see that the Cartographer still has a problem with "unwrapping" the Laser/PointCloud data and constructing the map. This can be noticeable from the deformation od the walls, as well as the blurred-image of the recycling bin. 

    




## Running attempt #3: Laser data, no Odometer, slow path around the main lab room


The conversion to Laser data was made using the [depthimage-to-laserscan](https://wiki.ros.org/depthimage_to_laserscan) package.

Install:
```
sudo apt-get install ros-melodic-depthimage-to-laserscan
```

Then, add it to the end of the RealSense launch file:
```
	<node name="depthimage_to_laserscan" pkg="depthimage_to_laserscan" type="depthimage_to_laserscan" > 
  	<remap from="image" to="/camera/depth/image_rect_raw"/>
	</node>
```


**Depth image type:**  Laser scan, produced by RealSense depth image and converted using depthimage_to_laserscan.

**Odometry used:** No odometry data (Bitcraze outputs garbage values). This seems to be the major cause of failure, as will be described

**Mapping type (2D/3D):** 2D


**More info:** 
The Robot was first put on a wheeled-rotating-chair and taken around the lab. 
The reason for the chair was to make relatively-slow movement for the robot. 



**Results**
The laser scan seems to introduce less "noise", but the lack of odometry data is still taking its toll. 
It seems like although the Cartographer can produce good mapping for a "static point", it fails to sync images taken in different
locations into a coherent map. 


### Running attempt #4: Trying to fine-tune the algorithm min/max range
TRAJECTORY_BUILDER_nD.min_range
TRAJECTORY_BUILDER_nD.max_range



















