

#ifdef ROS_COMPILATION


#include "ros_lib.h"
#include "sensor_msgs/point_cloud2_iterator.h"


sensor_msgs::PointCloud2 RosIntegration::PointCloudConversion(rs2::points pc) 
{
    // Take timestamp
    auto tsf = ros::Time::now();
    auto pc_size = static_cast<int>(pc.size());
    
  
    
    // Init ROS PointCloud2 message
    sensor_msgs::PointCloud2 msg_pointcloud;
    msg_pointcloud.header.stamp = tsf; // TODO, maybe use frame timestamp (if processing takes long)
    msg_pointcloud.header.frame_id = "map"; // dgreenbe TODO change name
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












void BaseRealSenseNode::publishPointCloud(rs2::points pc, const ros::Time& t, const rs2::frameset& frameset)
{
    std::vector<NamedFilter>::iterator pc_filter = find_if(_filters.begin(), _filters.end(), [] (NamedFilter s) { return s._name == "pointcloud"; } );
    rs2_stream texture_source_id = static_cast<rs2_stream>(pc_filter->_filter->get_option(rs2_option::RS2_OPTION_STREAM_FILTER));
    bool use_texture = texture_source_id != RS2_STREAM_ANY;
    static int warn_count(0);
    static const int DISPLAY_WARN_NUMBER(5);
    rs2::frameset::iterator texture_frame_itr = frameset.end();
    
    

    int texture_width(0), texture_height(0);
    int num_colors(0);

    const rs2::vertex* vertex = pc.get_vertices();
    const rs2::texture_coordinate* color_point = pc.get_texture_coordinates();
    
    /* Filter invalid data */
    std::list<unsigned int> valid_indices;
    for (size_t point_idx=0; point_idx < pc.size(); point_idx++, vertex++, color_point++)
    {
        if (static_cast<float>(vertex->z) > 0)
        {
            float i = static_cast<float>(color_point->u);
            float j = static_cast<float>(color_point->v);
            if (_allow_no_texture_points || (i >= 0.f && i <= 1.f && j >= 0.f && j <= 1.f))
            {
                valid_indices.push_back(point_idx);
            }
        }
    }

    /* Prepare ROS message */
    sensor_msgs::PointCloud2 msg_pointcloud;
    msg_pointcloud.header.stamp = t;
    msg_pointcloud.header.frame_id = "map";
    msg_pointcloud.width = static_cast<int>(valid_indices.size());
    msg_pointcloud.height = 1;
    msg_pointcloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");    

    vertex = pc.get_vertices();
    if (use_texture)
    {
        rs2::video_frame texture_frame = (*texture_frame_itr).as<rs2::video_frame>();
        texture_width = texture_frame.get_width();
        texture_height = texture_frame.get_height();
        num_colors = texture_frame.get_bytes_per_pixel();
        uint8_t* color_data = (uint8_t*)texture_frame.get_data();
        std::string format_str;
        switch(texture_frame.get_profile().format())
        {
            case RS2_FORMAT_RGB8:
                format_str = "rgb";
                break;
            case RS2_FORMAT_Y8:
                format_str = "intensity";
                break;
            default:
                throw std::runtime_error("Unhandled texture format passed in pointcloud " + std::to_string(texture_frame.get_profile().format()));
        }
        msg_pointcloud.point_step = addPointField(msg_pointcloud, format_str.c_str(), 1, sensor_msgs::PointField::FLOAT32, msg_pointcloud.point_step);
        msg_pointcloud.row_step = msg_pointcloud.width * msg_pointcloud.point_step;
        msg_pointcloud.data.resize(msg_pointcloud.height * msg_pointcloud.row_step);

        sensor_msgs::PointCloud2Iterator<float>iter_x(msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(msg_pointcloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t>iter_color(msg_pointcloud, format_str);
        color_point = pc.get_texture_coordinates();

        float color_pixel[2];
        unsigned int prev_idx(0);
        for (auto idx=valid_indices.begin(); idx != valid_indices.end(); idx++)
        {
            unsigned int idx_jump(*idx-prev_idx);
            prev_idx = *idx;
            vertex+=idx_jump;
            color_point+=idx_jump;

            *iter_x = vertex->x;
            *iter_y = vertex->y;
            *iter_z = vertex->z;

            float i(color_point->u);
            float j(color_point->v);
            if (i >= 0.f && i <= 1.f && j >= 0.f && j <= 1.f)
            {
                color_pixel[0] = i * texture_width;
                color_pixel[1] = j * texture_height;
                int pixx = static_cast<int>(color_pixel[0]);
                int pixy = static_cast<int>(color_pixel[1]);
                int offset = (pixy * texture_width + pixx) * num_colors;
                reverse_memcpy(&(*iter_color), color_data+offset, num_colors);  // PointCloud2 order of rgb is bgr.
            }

            ++iter_x; ++iter_y; ++iter_z;
            ++iter_color;
        }
    }

    _pointcloud_publisher.publish(msg_pointcloud);
}









#endif /* ROS_COMPILATION */

