

#ifdef ROS_COMPILATION


#include "RealSenseAPI.h" 

#ifndef NO_CAMERA
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#endif

#include "basic_utils.h"
#include "ros_lib.h"

namespace RosIntegration
{


///////////////////////////////////////////
/////           DEFINITIONS           /////
///////////////////////////////////////////

CAMERA_DEFINITION
(

    typedef struct
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    } ColorPixel;    
)


///////////////////////////////////////////
/////            FUNCTIONS            /////
///////////////////////////////////////////

#ifdef ENABLE_EXPERIMENTAL_PCL_FUNCTIONS

CAMERA_DEFINITION
(

    sensor_msgs::PointCloud2 PointCloudConversion(rs2::points pc) 
    {
        // Take timestamp
        auto tsf = ros::Time::now();
        auto pc_size = static_cast<int>(pc.size());
        
      
        
        // Init ROS PointCloud2 message
        sensor_msgs::PointCloud2 msg_pointcloud;
        
        msg_pointcloud.header.stamp = tsf; // TODO, maybe use frame timestamp (if processing takes long)
        msg_pointcloud.header.frame_id = tf_realsense_frame_id; 
        msg_pointcloud.width = pc_size; // This will be updated after filtering out bad points
        msg_pointcloud.height = 1;
        msg_pointcloud.is_dense = false;

        // Define builder for PointCloud message
        sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");    
        
        sensor_msgs::PointCloud2Iterator<float>iter_x(msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(msg_pointcloud, "z");
        
        // Fill ROS message with frame data 
        const rs2::vertex* vertex_array = pc.get_vertices();
        
        for (auto i=0; i < pc.size(); i++)
        {
            // Get vertex
            const rs2::vertex* current_vertex = &vertex_array[i];

            // Update message
            *iter_x = current_vertex->x;
            *iter_y = current_vertex->y;
            *iter_z = current_vertex->z;
            
            // Update builder iterators
            ++iter_x; ++iter_y; ++iter_z;
        }
        
        return msg_pointcloud;
    }




)

#endif  // ENABLE_EXPERIMENTAL_PCL_FUNCTIONS



} // namespace RosIntegration



#endif /* ROS_COMPILATION */

