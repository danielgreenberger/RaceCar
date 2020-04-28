#ifndef _ROS_JETSON_TF_PUBLISHER_H_
#define _ROS_JETSON_TF_PUBLISHER_H_

/*
	@DESCRIPTION
	
	This file defines a special type of ROS publisher intended for publishing odometer data. 
    In addition to being a child class of ros_publisher, this class also utilizes the TF 
    library to define and maintain different coordinate systems for the odometer, robot baselink
    and a static world frame-of-reference (i.e Lab starting point). 
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



class OdometerPublisher : public Publisher<nav_msgs::Odometry>
{
    
//--------------------------------------------------------------------------
//                             Members
//--------------------------------------------------------------------------
private:
    
    /* 
            Static transforms publisher 
            
            Used for publishing transforms that are not expected to change at run-time, 
            such as  Odometer => Camera frames transform 
    */
    std::shared_ptr<tf::TransformBroadcaster> m_p_static_broadcaster;  
    std::shared_ptr<tf::TransformListener> m_p_listener;
    
    
    /*
            Odometer data accumulator
            
            
    */
    
    using dist_t = int;
    using tsf_t = int;
    // TODO: we must protect against overflow or wraparound
    dist_t m_total_dist_x = 0;
    dist_t m_total_dist_y = 0;
    dist_t m_curr_height_z = 0;
    tsf_t m_last_odom_timestamp = 0;   
    #warning "TODO: make sure all members are needed"
    


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
    OdometerPublisher (string topic_name, uint32_t flags = DEFAULT_FLAGS, uint32_t max_queue_size = ROS_TOPIC_QUEUE_BUFFER_SIZE)
        :
        Publisher<nav_msgs::Odometry> (topic_name, INVALID_NODE_NAME, flags, max_queue_size)
    {
        
    }
    
    
    
    
    
    

//--------------------------------------------------------------------------
//                           Publisher interface functions
//--------------------------------------------------------------------------

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
void publish(const Flow& odom_data)
{
    nav_msgs::Odometry ros_odom = convert_to_ros_odom(odom_data);
    Publisher::publish(ros_odom);
}

//--------------------------------------------------------------------------
//                           Publisher internal functions
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
        ros::Publisher node_publisher = node_handler.advertise<nav_msgs::Odometry>(this->get_topic_name(), 100);

        /* Init TF publisher and listener */
        m_p_listener = std::make_shared<tf::TransformListener> (ros::Duration(10));
        m_p_static_broadcaster = std::make_shared<tf::TransformBroadcaster> ();  // TODO: publisher/listener should belong to node
        
        /* Define publish frequency */
        ros::Rate publisher_rate_per_second(10);
        const int tf_rate_loop = 10;
        
        
        /* Publish in loop */
        int count = 0;
        while ( this->active() )
        {
            
           
            publish_static_transforms();
            
            // Pull a message from FIFO
            nav_msgs::Odometry msg;
            bool msg_exists = this->pull_message_if_exists(msg);
            
                
            // Publish
            if (msg_exists)
            {				
                ROS_INTEGRATION_DEBUG_PRINT("Publishing message: " << ++count);  
                 publish_odom_to_lab_transformation(msg);
                node_publisher.publish(msg);
                ros::spinOnce();
            }


            // Wait for next publish interval 
            publisher_rate_per_second.sleep();
            
        }
        
        
        
        
        
        
    }
    
    
    
    
    
    /*=======================================================
    * @description     Publish coordinate frames data between the robots internal components (Camera, odomeder etc.)
    *                  which remain constant during runtime
    *
    * @return          None.
    *
    * @author          Daniel Greenberger
    =========================================================*/
    void publish_static_transforms()
    {
        
        ASSERT(m_p_static_broadcaster != nullptr, std::exception());
        
        /* Transform:  Base -> Camera */
        auto transform = tf::Transform(RSENSE_TO_BASE_ROTATION, RSENSE_TO_BASE_TRANSLATION);
        auto tf_transform = tf::StampedTransform(
                                                transform,
                                                ros::Time::now(),
                                                BASE_FRAME_ID,
                                                CAMERA_FRAME_ID
                                                );
        m_p_static_broadcaster->sendTransform(tf_transform);
                                      

        /* Transform:  Camera -> Bitcraze */
        transform = tf::Transform(BCRAZE_TO_RSENSE_ROTATION, BCRAZE_TO_RSENSE_TRANSLATION);
        tf_transform = tf::StampedTransform(
                                              transform,
                                              ros::Time::now(),
                                              CAMERA_FRAME_ID,
                                              ODOMETER_FRAME_ID 
                                              );
        m_p_static_broadcaster->sendTransform(tf_transform);

    }
    
    
    
    /*=======================================================
    * @description     Publish the current transformation between the RaceCar odometry frame and the lab,
    *                  according to the accumulated data from the odometer.
    * 
    * @param           odom_msg  -  This is an odometry message which will be used to define the transformation 
    *                               We will use the xyz position of the message to construct our translation, 
    *                               and the twist will be always 0
    * 
    *                  TODO: maybe add twist as well?
    * 
    * @author          Daniel Greenberger
    =========================================================*/
    void publish_odom_to_lab_transformation(nav_msgs::Odometry odom_msg)
    {
        ASSERT(m_p_static_broadcaster != nullptr, std::exception());

        
        const auto curr_translation_vec = tf::Vector3(
                                                      odom_msg.pose.pose.position.x,
                                                      odom_msg.pose.pose.position.y,
                                                      odom_msg.pose.pose.position.z
                                                   );
        auto transform = tf::Transform(tf::Matrix3x3::getIdentity(), curr_translation_vec);

        auto tf_transform = tf::StampedTransform(
                                              transform,
                                              odom_msg.header.stamp,
                                              ODOMETER_FRAME_ID,
                                              LAB_FRAME_ID 
                                              );
        m_p_static_broadcaster->sendTransform(tf_transform);
    
        
    }
    
    
    /*=======================================================
    * @brief           Converts Bitcraze data msg to standard ROS odometry msg
    *
    * @param           odom_data  -  Odometry in Bitcraze format
    *
    * @return          ROS odometry msg, which can be published to a topic
    *
    * @author          Daniel Greenberger
    =========================================================*/
    nav_msgs::Odometry convert_to_ros_odom(const Flow& odom_data)
    {
        /* Update total distance counter */
        m_total_dist_x += odom_data.deltaX;
        m_total_dist_y += odom_data.deltaY;
        
        
        /* Calculate speed according to last result */
        const dist_t speed_x = (odom_data.deltaX) / (odom_data.dt);
        const dist_t speed_y = (odom_data.deltaY) / (odom_data.dt);
        
        
        /* Build ROS odometry msg */
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = ODOMETER_FRAME_ID;
        
        odom.pose.pose.position.x = m_total_dist_x;
        odom.pose.pose.position.y = m_total_dist_y;
        odom.pose.pose.position.z = odom_data.range;
        
        odom.child_frame_id = LAB_FRAME_ID;
        odom.twist.twist.linear.x = speed_x;
        odom.twist.twist.linear.y = speed_y;
        
        return odom;
        
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
