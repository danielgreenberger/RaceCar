#ifndef _ROS_JETSON_UTILS_H_
#define _ROS_JETSON_UTILS_H_

/*
	@DESCRIPTION
	
	TODO
*/

#ifdef ROS_COMPILATION




// ROS API
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"

// RosIntegration
#include "ros_defs.h"




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

#ifndef NO_CAMERA
    /*=======================================================
    * @brief           Converts RealSense pointcloud to ROS PointCloud2 message
    *
    * @param           pc         -  RealSense point cloud.
    *
    * @return          ROS PointCloud2 msg
    =========================================================*/
    sensor_msgs::PointCloud2 PointCloudConversion(rs2::points pc);
    sensor_msgs::PointCloud2 PointCloudConversion(rs2::points pc, const rs2::video_frame color_frame);

#endif





}




#endif /* ROS_COMPILATION */


#endif /*_ROS_JETSON_UTILS_H_*/
