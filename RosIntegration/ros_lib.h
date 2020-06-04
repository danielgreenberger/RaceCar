#ifndef _ROS_JETSON_UTILS_H_
#define _ROS_JETSON_UTILS_H_

/*
    @AUTHOR  Daniel Greenberger  (david.daniel.greenberger@gmail.com)
    
	@DESCRIPTION
	
	This file contains miscellaneous library functions which can be used with under ROS.
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


/*
*      Experimental functions
*     
*      For some functions in this file, we were not able to validate their input. 
*      We decided to still include them under compilation flag, in hopes the code 
*      could still be useful at some point in the future. 
*                         
*      BEFORE USING THE FUNCTIONS BELOW, MAKE SURE TO VALIDATE THAT POINTCLOUD REPRESENTS A VALID OUTPUT.     
*
*/
//#define ENABLE_EXPERIMENTAL_PCL_FUNCTIONS


/////////////////////////////////////////
/////           FUNCTIONS           /////
/////////////////////////////////////////

namespace RosIntegration
{

	

#ifdef ENABLE_EXPERIMENTAL_PCL_FUNCTIONS
    #ifndef NO_CAMERA
    #warn "The outout of PointCloudConversion has not been verified."
        /*=======================================================
        * @brief           Converts RealSense pointcloud to ROS PointCloud2 message
        *
        * @param           pc         -  RealSense point cloud.
        *
        * @return          ROS PointCloud2 msg
        =========================================================*/
        sensor_msgs::PointCloud2 PointCloudConversion(rs2::points pc);
        sensor_msgs::PointCloud2 PointCloudConversion(rs2::points pc, const rs2::video_frame color_frame);

    #endif  /* NO_CAMERA */
#endif  /* ENABLE_EXPERIMENTAL_PCL_FUNCTIONS */





}




#endif /* ROS_COMPILATION */


#endif /*_ROS_JETSON_UTILS_H_*/
