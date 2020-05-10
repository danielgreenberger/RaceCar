

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
If you haven't yet installed google-cartographer, see the [offical guide](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation) for more details. 
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


### Running attempt #1: PointCloud2, no Odometer

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
However, looking the zoom-in image (second figure) was can see the corridor. 





### Running attempt #2: Laser data, no Odometer, slow path around the main lab room


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
The laser scan seems to introduce less "noise", but the lack of odometry data has taken its toll. 
It seems like although the Cartographer can produce good mapping for a "static point", it fails to sync images taken in different
locations into coherent map. 
 



















