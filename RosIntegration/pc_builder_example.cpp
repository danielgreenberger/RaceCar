  #include <sensor_msgs/point_cloud_iterator.h>
  
  ///////////////////////////////////////////////////////////////
  //         Create PointCloud2 msg and define fields          //
  ///////////////////////////////////////////////////////////////
  
  // Create a PointCloud2
  sensor_msgs::PointCloud2 cloud_msg;
  
  // Fill some internals of the PoinCloud2 like the header/width/height ...
  cloud_msgs.height = 1;  
  cloud_msgs.width = 4;
  
  
  // Set the point fields to xyzrgb and resize the vector with the following command
  // 4 is for the number of added fields. Each come in triplet: the name of the PointField,
  // the number of occurrences of the type in the PointField, the type of the PointField
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2Fields(
                                    4,     
                                    "x", 1, sensor_msgs::PointField::FLOAT32,
                                    "y", 1, sensor_msgs::PointField::FLOAT32,
                                    "z", 1, sensor_msgs::PointField::FLOAT32,
                                    "rgb", 1, sensor_msgs::PointField::FLOAT32
                               );
                                           

  
  // You can then reserve / resize as usual
  modifier.resize(100);
  
  
  ////////////////////////////////////////////////////////////
  //         traverse PointCloud using an iterator          //
  ////////////////////////////////////////////////////////////

  // Define some raw data we'll put in the PointCloud2
  float point_data[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
  uint8_t color_data[] = {40, 80, 120, 160, 200, 240, 20, 40, 60, 80, 100, 120};
  
  // Define the iterators. When doing so, you define the Field you would like to iterate upon and
  // the type of you would like returned: it is not necessary the type of the PointField as sometimes
  // you pack data in another type (e.g. 3 uchar + 1 uchar for RGB are packed in a float)
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  
  // Even though the r,g,b,a fields do not exist (it's usually rgb, rgba), you can create iterators for
  // those: they will handle data packing for you (in little endian RGB is packed as *,R,G,B in a float
  // and RGBA as A,R,G,B)
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");
  
  
  // Fill the PointCloud2
  for(size_t i=0; i<n_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
    *iter_x = point_data[3*i+0];
    *iter_y = point_data[3*i+1];
    *iter_z = point_data[3*i+2];
    *iter_r = color_data[3*i+0];
    *iter_g = color_data[3*i+1];
    *iter_b = color_data[3*i+2];
  }