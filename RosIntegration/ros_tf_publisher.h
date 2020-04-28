#ifndef _ROS_JETSON_TF_PUBLISHER_H_
#define _ROS_JETSON_TF_PUBLISHER_H_

/*
	@DESCRIPTION
	
	TODO
*/

#ifdef ROS_COMPILATION

#include <string>
#include <queue>
#include <sstream>
#include <iostream>
#include <thread>
#include "ros_defs.h"
#include "basic_utils.h"
#include "ros_publisher.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include "racecar_coordinates.h"
#include <nav_msgs/Odometry.h>

namespace RosIntegration
{
using string = std::string;

////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                           MACROS                                             /////
////////////////////////////////////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                  PUBLISHER  CONFIGURATION                                    /////
////////////////////////////////////////////////////////////////////////////////////////////////////////









////////////////////////////////////////////////////////////////////////////////////////////////
/////                                  PUBLISHER  CLASS                                    /////
////////////////////////////////////////////////////////////////////////////////////////////////


template <typename T>
class TimeframePublisher : public Publisher<T>
{
private:
    std::shared_ptr<tf::TransformBroadcaster> m_p_broadcaster;  // TODO: consider moving both back to inside function
    std::shared_ptr<tf::TransformListener> m_p_listener;
    
    


public:    
   /*=======================================================
    * @brief           Simple constructor for the publisher class
    *
    * @description     This is the simplest constructor for the publisher class. 
    *                  The node name will be auto-generated according to the topic name.
    *
    * @param           topic_name       -  Name of the topic for which the messages will be published.
    *
    * @param           flags            - (Optional) configuration flags for publisher class.
    *
    * @param           max_queue_size   - (Optional) Maximum number of outgoing messages to be queued for delivery to subscribers.
    *
    * @author          Daniel Greenberger
    =========================================================*/
    TimeframePublisher (string topic_name, uint32_t flags = DEFAULT_FLAGS, uint32_t max_queue_size = ROS_TOPIC_QUEUE_BUFFER_SIZE)
        :
        Publisher<T> (topic_name, INVALID_NODE_NAME, flags, max_queue_size)
    {
        
    }
    
    
    
    

//--------------------------------------------------------------------------
//                           Publisher interface functions
//--------------------------------------------------------------------------
private:
    /*=======================================================
    * @brief           Publisher node main loop
    *
    * @description     This function works in an infinate loop while the publisher is active. 
    *                  We are checking the internal object FIFO for new messages. 
    *                  When a new message exists, it is published to the ROS topic
    *                  which was pre-defined for this object.
    *
    *
    * @param           node_handler  -  This is the access point to the node functionality. 
    *
    * @return          None.
    *
    * @author          Daniel Greenberger
    =========================================================*/
    virtual void __publisher_loop(ros::NodeHandle& node_handler) override
    {
        
        /* Register to publish for topic */
        ros::Publisher node_publisher = node_handler.advertise<T>(this->get_topic_name(), 100);

        /* Init TF publisher and listener */
        m_p_listener = std::make_shared<tf::TransformListener> (ros::Duration(10));
      //  m_p_broadcaster = std::make_shared<tf::TransformBroadcaster> ();  // TODO: publisher/listener should belong to node

        
        /* Define publish frequency */
        ros::Rate publisher_rate_per_second(10);
        const int tf_rate_loop = 10;
        
        /* Publish in loop */
        int count = 0;
        while ( this->active() )
            
        {
            
            publish_frame_transform();
            
            // Pull a message from FIFO
            T msg;
            bool msg_exists = this->pull_message_if_exists(msg);
            
                
            // Publish
            if (msg_exists)
            {				
                ROS_INTEGRATION_DEBUG_PRINT("Publishing message: " << ++count);  
                
                node_publisher.publish(msg);
                ros::spinOnce();
            }


            // Wait for next publish interval 
            publisher_rate_per_second.sleep();
            
       //     transformPoint();
            
        }
        
        
        
        
        
        
    }
    
    
    
    
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
    void publish_frame_transform()
    {
        
        m_p_broadcaster = std::make_shared<tf::TransformBroadcaster> ();  // TODO: publisher/listener should belong to node
       
        std::cout << "Publishing TF transform" <<std::endl;
        
        /* Transform:  Base -> Camera */
        auto transform = tf::Transform(RSENSE_TO_BASE_ROTATION, RSENSE_TO_BASE_TRANSLATION);
        auto tf_transform = tf::StampedTransform(
                                                transform,
                                                ros::Time::now(),
                                                BASE_FRAME_ID,
                                                CAMERA_FRAME_ID
                                                );
        m_p_broadcaster->sendTransform(tf_transform);
                                      

        /* Transform:  Camera -> Bitcraze */
        transform = tf::Transform(BCRAZE_TO_RSENSE_ROTATION, BCRAZE_TO_RSENSE_TRANSLATION);
        tf_transform = tf::StampedTransform(
                                              transform,
                                              ros::Time::now(),
                                              CAMERA_FRAME_ID,
                                              ODOMETER_FRAME_ID 
                                              );
        m_p_broadcaster->sendTransform(tf_transform);

    }


//--------------------------------------------------------------------------
//                           Publisher interface functions
//--------------------------------------------------------------------------
//      void transformPoint()
//      {
//         // We'll create a point in the odometer frame that we'd like to transform to the base frame
//         geometry_msgs::PointStamped odom_point;
//         
//         odom_point.header.frame_id = ODOMETER_FRAME_ID;
//         odom_point.header.stamp = ros::Time();
//         odom_point.point.x = 1.0;
//         odom_point.point.y = 1.0;
//         odom_point.point.z = 1.0;
//   
//   
//         try
//         {
//          geometry_msgs::PointStamped base_point;
//          m_p_listener->transformPoint(CAMERA_FRAME_ID, odom_point, base_point);
//   
//          ROS_INFO("odometer: (%.2f, %.2f. %.2f) -----> base: (%.2f, %.2f, %.2f) at time %.2f",
//              odom_point.point.x, odom_point.point.y, odom_point.point.z,
//              base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
//         }
//         catch(tf::TransformException& ex)
//         {
//              ROS_ERROR("Received an exception trying to transform a point from \"odometer\" to \"base\": %s", ex.what());
//              ASSERT(false, tf::TransformException("Received an exception trying to transform a point"));
//         }
//      }

#undef ROS_PUBLISHER_CRITICAL_SECTION



};

}


#endif /* ROS_COMPILATION */


#endif /*_ROS_JETSON_TF_PUBLISHER_H_*/
