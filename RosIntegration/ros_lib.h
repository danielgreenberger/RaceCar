#ifndef _ROS_JETSON_UTILS_H_
#define _ROS_JETSON_UTILS_H_

/*
	@DESCRIPTION
	
	TODO
*/

#ifdef ROS_COMPILATION

//#include <string>
//#include <queue>
//#include <sstream>
//#include <iostream>
//#include <thread>
#include <librealsense2/rs.hpp>
#include "ros_defs.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"





/////////////////////////////////////
/////           MACROS          /////
/////////////////////////////////////




/////////////////////////////////////////
/////           FUNCTIONS           /////
/////////////////////////////////////////

namespace RosIntegration
{
	
/*
dgreenbe TODO:
    1. Make this a conversion function from RealSense point cloud into ROS point cloud

*/


/*=======================================================
* @brief           Converts RealSense pointcloud to ROS PointCloud2 message
*
* @param           pc         -  RealSense point cloud.
*
* @return          ROS PointCloud2 msg
=========================================================*/
sensor_msgs::PointCloud2 PointCloudConversion(rs2::points pc);



}




#endif /* ROS_COMPILATION */


#endif /*_ROS_JETSON_UTILS_H_*/
