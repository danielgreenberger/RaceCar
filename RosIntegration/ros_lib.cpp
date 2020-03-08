

#ifdef ROS_COMPILATION

// RealSense API
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
//#include <rs_frame.hpp>



// RosIntegration
#include "ros_lib.h"

namespace RosIntegration
{


///////////////////////////////////////////
/////           DEFINITIONS           /////
///////////////////////////////////////////


// dgreenbe TODO explain
static const std::string tf_realsense_frame_id = "map";


// dgreenbe todo check if struct exists else where
typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} ColorPixel;    



///////////////////////////////////////////
/////            FUNCTIONS            /////
///////////////////////////////////////////


/*=======================================================
* @brief           TODO
*
* @description     TODO
*
* @param           TODO
*
* @return          TODO
*
* @author          TODO
=========================================================*/
// dgreenbe todo inline
static inline uint8_t* GetColorPixel(const rs2::video_frame color_frame, const rs2::texture_coordinate* texture_point)
{
    _ASSERT(texture_point, todo);
    
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
    _ASSERT(texture_valid, todo);
   
   
    /* Calculate pixel offset in data*/
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






//  
//  
//  
//  void BaseRealSenseNode::publishPointCloud(rs2::points pc, const ros::Time& t, const rs2::frameset& frameset)
//  {
//      /* Init */
//      static int warn_count(0);
//      static const int DISPLAY_WARN_NUMBER(5);
//      rs2::frameset::iterator texture_frame_itr = frameset.end();
//      
//      int texture_width(0), texture_height(0);
//      
//      
//      
//      /* Check if to use texture */
//      std::vector<NamedFilter>::iterator pc_filter = find_if(_filters.begin(), _filters.end(), [] (NamedFilter s) { return s._name == "pointcloud"; } );
//      
//      rs2_stream texture_source_id = static_cast<rs2_stream>(pc_filter->_filter->get_option(rs2_option::RS2_OPTION_STREAM_FILTER));
//      bool use_texture = texture_source_id != RS2_STREAM_ANY;
//      
//      
//      /* Filter invalid data */
//      const rs2::vertex* vertex = pc.get_vertices();     // Points
//      const rs2::texture_coordinate* color_point = pc.get_texture_coordinates();  // Texture
//      std::list<unsigned int> valid_indices;
//      
//      for (size_t point_idx = 0; point_idx < pc.size(); point_idx++, vertex++, color_point++)
//      {
//          if (static_cast<float>(vertex->z) > 0)
//          {
//              float i = static_cast<float>(color_point->u);
//              float j = static_cast<float>(color_point->v);
//              if (_allow_no_texture_points || (i >= 0.f && i <= 1.f && j >= 0.f && j <= 1.f))
//              {
//                  valid_indices.push_back(point_idx);
//              }
//          }
//      }
//  
//      /* Prepare ROS message */
//      sensor_msgs::PointCloud2 msg_pointcloud;
//      msg_pointcloud.header.stamp = t;
//      msg_pointcloud.header.frame_id = "map";
//      msg_pointcloud.width = static_cast<int>(valid_indices.size());
//      msg_pointcloud.height = 1;
//      msg_pointcloud.is_dense = true;
//  
//      sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);
//      modifier.setPointCloud2FieldsByString(1, "xyz");    
//  
//      vertex = pc.get_vertices();
//      if (use_texture)
//      {
//          rs2::video_frame texture_frame = (*texture_frame_itr).as<rs2::video_frame>();
//          
//          texture_width = texture_frame.get_width();    // Image width in pixels
//          texture_height = texture_frame.get_height();  // Image height in pixels
//          
//          color_bytes_per_pixel = texture_frame.get_bytes_per_pixel(); 
//          
//          uint8_t* color_data = (uint8_t*)texture_frame.get_data();
//          
//          /* Get color format */
//          std::string format_str;
//          switch(texture_frame.get_profile().format())
//          {
//              case RS2_FORMAT_RGB8:
//                  format_str = "rgb";
//                  break;
//              case RS2_FORMAT_Y8:
//                  format_str = "intensity";
//                  break;
//              default:
//                  throw std::runtime_error("Unhandled texture format passed in pointcloud " + std::to_string(texture_frame.get_profile().format()));
//          }
//          
//          msg_pointcloud.point_step = addPointField(msg_pointcloud, format_str.c_str(), 1, sensor_msgs::PointField::FLOAT32, msg_pointcloud.point_step);
//          msg_pointcloud.row_step = msg_pointcloud.width * msg_pointcloud.point_step;
//          msg_pointcloud.data.resize(msg_pointcloud.height * msg_pointcloud.row_step);
//  
//          sensor_msgs::PointCloud2Iterator<float>iter_x(msg_pointcloud, "x");
//          sensor_msgs::PointCloud2Iterator<float>iter_y(msg_pointcloud, "y");
//          sensor_msgs::PointCloud2Iterator<float>iter_z(msg_pointcloud, "z");
//          sensor_msgs::PointCloud2Iterator<uint8_t>iter_color(msg_pointcloud, format_str);
//          
//          color_point = pc.get_texture_coordinates();
//  
//          float color_pixel[2];
//          unsigned int prev_idx(0);
//          for (auto idx=valid_indices.begin(); idx != valid_indices.end(); idx++)
//          {
//              unsigned int idx_jump(*idx-prev_idx);
//              prev_idx = *idx;
//              vertex+=idx_jump;
//              color_point+=idx_jump;
//  
//              *iter_x = vertex->x;
//              *iter_y = vertex->y;
//              *iter_z = vertex->z;
//  
//              float i(color_point->u);
//              float j(color_point->v);
//              if (i >= 0.f && i <= 1.f && j >= 0.f && j <= 1.f)
//              {
//                  color_pixel[0] = i * texture_width;
//                  color_pixel[1] = j * texture_height;
//
//                  int pixx = static_cast<int>(color_pixel[0]);
//                  int pixy = static_cast<int>(color_pixel[1]);
//                  int offset = (pixy * texture_width + pixx) * color_bytes_per_pixel;
//                  reverse_memcpy(&(*iter_color),  +offset, color_bytes_per_pixel);  // PointCloud2 order of rgb is bgr.
//              }
//  
//              ++iter_x; ++iter_y; ++iter_z;
//              ++iter_color;
//          }
//      }
//  
//      _pointcloud_publisher.publish(msg_pointcloud);
//  }
//  
//  
//  




} // namespace RosIntegration



#endif /* ROS_COMPILATION */

