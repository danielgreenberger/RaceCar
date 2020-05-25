

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

CAMERA_DEFINITION
(

    static inline uint8_t* GetColorPixel(const rs2::video_frame color_frame, const rs2::texture_coordinate* texture_point)
    {
        ASSERT(texture_point, std::exception());
        
        /* Get color frame metadata */
        uint32_t color_width = color_frame.get_width();    // Image width in pixels
        uint32_t color_height = color_frame.get_height();  // Image height in pixels
        uint32_t color_bytes_per_pixel = color_frame.get_bytes_per_pixel(); 
               
        /* Get color frame data */
        uint8_t* color_data = (uint8_t*)color_frame.get_data(); 
        
        /* Get texture indices */
        // dgreenbe todo explain meaning of i and j
        float i(texture_point->u);
        float j(texture_point->v);
        
        bool texture_valid = (i >= 0.f && i <= 1.f && j >= 0.f && j <= 1.f);
        ASSERT(texture_valid, std::exception());
       
       
        /* Calculate pixel offset in data */
        if (texture_valid) 
        {
            int pixel_x = static_cast<int>(i * color_width);
            int pixel_y = static_cast<int>(j * color_height);
            
            int base_offset_pixels = (pixel_y * color_width) + pixel_x;
            int base_offset_bytes = base_offset_pixels * color_bytes_per_pixel;
            
            return color_data + base_offset_bytes;
        }
        else
        {
            return NULL;
        }
    }


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


    sensor_msgs::PointCloud2 PointCloudConversion(rs2::points pc, const rs2::video_frame color_frame)
    {  

          /////////////////////////////////////////////////////
          //         Get realsense PointCloud data           //
          /////////////////////////////////////////////////////
          
          int total_points = static_cast<int>(pc.size());
          auto ros_tsf = ros::Time::now(); // dgreenbe TODO use frame tsf instead?
          
          const rs2::vertex* points_array = pc.get_vertices();
          const rs2::texture_coordinate* texture_points_array = pc.get_texture_coordinates();
          
          
          ///////////////////////////////////////////////////////////////
          //         Create PointCloud2 msg and define fields          //
          ///////////////////////////////////////////////////////////////
          
          // Create a PointCloud2
          sensor_msgs::PointCloud2 pcl_msg;
          
          // Fill some internals of the PoinCloud2 like the header/width/height ...
          pcl_msg.height = 1;  
          pcl_msg.width = total_points;
          pcl_msg.header.stamp = ros_tsf; 
          pcl_msg.header.frame_id = tf_realsense_frame_id; 
          pcl_msg.is_dense = false;
          
          
          // Set the point fields to xyzrgb and resize the vector with the following command.
          // 4 is for the number of added fields. 
          // Each field comes in triplet: 
          //      (1) the name of the PointField,
          //      (2) the number of occurrences of the type in the PointField
          //      (3) the type of the PointField
          sensor_msgs::PointCloud2Modifier modifier(pcl_msg);
          modifier.setPointCloud2Fields(
                                            4,     
                                            "x", 1, sensor_msgs::PointField::FLOAT32,
                                            "y", 1, sensor_msgs::PointField::FLOAT32,
                                            "z", 1, sensor_msgs::PointField::FLOAT32,
                                            "rgb", 1, sensor_msgs::PointField::FLOAT32
                                       );
                                                   

          
          
          ////////////////////////////////////////////////////////////
          //         traverse PointCloud using an iterator          //
          ////////////////////////////////////////////////////////////

          // Define the iterators
          sensor_msgs::PointCloud2Iterator<float> iter_x(pcl_msg, "x");
          sensor_msgs::PointCloud2Iterator<float> iter_y(pcl_msg, "y");
          sensor_msgs::PointCloud2Iterator<float> iter_z(pcl_msg, "z");
          
          // Even though the r,g,b,a fields do not exist (it's usually rgb, rgba), we can create iterators for
          // them. they will handle data packing for you (in little endian RGB is packed as *,R,G,B in a float
          // and RGBA as A,R,G,B)
          sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pcl_msg, "r");
          sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pcl_msg, "g");
          sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pcl_msg, "b");
          
          
          // Fill the PointCloud2
          for(size_t i=0; i<total_points; ++i) 
          {
            const rs2::vertex* current_point = &points_array[i];
            const rs2::texture_coordinate* current_point_texture = &texture_points_array[i];
            
            // Filter bad points
            // dgreenbe TODO
          
            // Fill point data
            *iter_x = current_point->x;
            *iter_y = current_point->y;
            *iter_z = current_point->z;
            
            // Fill color data
            uint8_t* current_pixel_p;
            const ColorPixel* current_pixel = reinterpret_cast<const ColorPixel*>(GetColorPixel(color_frame, current_point_texture));
            *iter_r = current_pixel->r;
            *iter_g = current_pixel->g;
            *iter_b = current_pixel->g;
            
            // Advance iterators
            ++iter_x; 
            ++iter_y; 
            ++iter_z; 
            ++iter_r; 
            ++iter_g; 
            ++iter_b;
          }
          
          /* Resize */
          // TODO after adding filter
          // modifier.resize(valid_points); // dgreenbe TODO is necessarry?
          
          /* Finish */
          return pcl_msg;
          
    }

)





} // namespace RosIntegration



#endif /* ROS_COMPILATION */

