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
#include "odometer_calculations.h"
#include <nav_msgs/Odometry.h>

namespace RosIntegration
{
using string = std::string;

////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                           MACROS                                             /////
////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
         Critical section macro. 
         Must only be used inside the OdometerPublisher class.
         
         A critical section is needed here, since we have to make sure that the accumelated odometer data is not getting updated
         while we are publishing it to ROS. 
*/
#define ODOMETER_CRITICAL_SECTION(code)   do{  __CRITICAL_SECTION(m_odometer_mutex, code)  } while(0);     



////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                  PUBLISHER  CONFIGURATION                                    /////
////////////////////////////////////////////////////////////////////////////////////////////////////////









////////////////////////////////////////////////////////////////////////////////////////////////
/////                                  PUBLISHER  CLASS                                    /////
////////////////////////////////////////////////////////////////////////////////////////////////


#warning "TODO remove inheritance"
class OdometerPublisher : public Publisher<nav_msgs::Odometry>
{
    
//--------------------------------------------------------------------------
//                             OdometerData struct
//--------------------------------------------------------------------------

struct OdometerData
{
     // TODO: we must protect against overflow or wraparound
     // TODO: make sure we only start publishing when we get actual odometer info (from bitcraze)
    
    /* Robot offset in X-Y  lab frame */
    double total_dist_x = 0;    
    double total_dist_y = 0;
    
    /* Robot rotation relative to X-Y lab axis */
    double total_rotation_z_axis = 0;
    
    /* Current height of the sensor from the ground in meters  -  no effect on robot rotation */
    double current_z_range = 0;
    
    /* Current speed */
    double current_speed_x = 0;  /* Speed in the X direction (m/s) */
    double current_speed_y = 0;  /* Speed in the Y direction (m/s) */
    
    /* time */
    ros::Time last_update_timestamp;
};



//--------------------------------------------------------------------------
//                             Members
//--------------------------------------------------------------------------
private:
    
    /* 
            Static transforms publisher 
            
            Used for publishing transforms that are not expected to change at run-time, 
            such as  Odometer => Camera frames transform 
    */
    std::shared_ptr<tf::TransformBroadcaster> m_p_TF_broadcaster;  
    std::shared_ptr<tf::TransformListener> m_p_TF_listener;
    
    
    /*
            Odometer data accumulator
            
    */
     OdometerData m_last_odom_snapshot;
    
    
    /* Mutex */
    std::mutex m_odometer_mutex; // See comment at critical-section macro about the need of the mutex.

    
    


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
    
    

    void publish(const Flow& odom_data) = delete;
    


//--------------------------------------------------------------------------
//                           Publisher interface functions
//--------------------------------------------------------------------------
public:

    
    
   /*=======================================================
    * @brief           Updates the accumelated distance and rotation according to the new Odometer data unit
    *
    * @param           odom_data  -  Odometry in Bitcraze format
    *
    * @return          None.
    *
    * @author          Daniel Greenberger
    =========================================================*/
    void update_odometer_data(const OdometerTransform::OdometerDataUnit& odom_data)
    {
        /* Get current time */
        const auto curr_time = ros::Time::now();
        

        /* Calculate added rotation around the z-axis */
        const double rotation_z_diff =  OdometerTransform::get_angle_from_linear_offset(odom_data.dx__meters, odom_data.dy__meters);

        
        /* Calculate added distance offset */
        OdometerTransform::OdometerDataUnit odom_lab_data;
        ODOMETER_CRITICAL_SECTION
        (
            
                // Note we are using the old rotation parameter to calculate the current offset.
                // This is because the current offset is the cause of the rotation.
                //
                // Since we assume deltaX, deltaY are small we can neglect the error 
                // arising from the fact that the change of the rotation is continous
                // TODO: maybe explain better
             
                 odom_lab_data = OdometerTransform::convert_odom_to_lab_frame( m_last_odom_snapshot.total_rotation_z_axis,  odom_data);  
               
               
            /* Update */
            m_last_odom_snapshot.total_dist_x += odom_lab_data.dx__meters;
            m_last_odom_snapshot.total_dist_y += odom_lab_data.dy__meters;
            
            m_last_odom_snapshot.current_speed_x = odom_lab_data.vx__meters_p_sec;
            m_last_odom_snapshot.current_speed_y = odom_lab_data.vy__meters_p_sec;
            
            m_last_odom_snapshot.last_update_timestamp = curr_time;
            m_last_odom_snapshot.current_z_range = odom_data.z_range__meters;
            m_last_odom_snapshot.total_rotation_z_axis += rotation_z_diff;
        )
        
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
        m_p_TF_listener = std::make_shared<tf::TransformListener> (ros::Duration(10));
        m_p_TF_broadcaster = std::make_shared<tf::TransformBroadcaster> ();  
        
        
        /* Define publish frequency */
        ros::Rate publisher_rate_per_second(10);
        
        
        /* Publish in loop */
        int count = 0;
        while ( this->active() )
        {
            // Publish  [Odometer => Base]  frame transformation 
            publish_static_TF_transforms();
            
            // Get current odometer snapshot
            OdometerData snapshot;
            ODOMETER_CRITICAL_SECTION
            (
                    snapshot = m_last_odom_snapshot;
            )
            
            // Publish TF transforms
            const auto tf_transform = get_odom_to_lab_TF_transform(snapshot);
            m_p_TF_broadcaster->sendTransform(tf_transform);
            
            
            // Publish Odom msg to topic
            const auto odom_msg = get_odom_to_lab_topic_msg(snapshot);
            node_publisher.publish(odom_msg);
            

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
    void publish_static_TF_transforms()
    {
        
        ASSERT(m_p_TF_broadcaster != nullptr, std::exception());
        
        /* Transform:  Base -> Camera */
        auto transform = tf::Transform(RSENSE_TO_BASE_ROTATION, RSENSE_TO_BASE_TRANSLATION);
        auto tf_transform = tf::StampedTransform(
                                                transform,
                                                ros::Time::now(),
                                                BASE_FRAME_ID,
                                                CAMERA_FRAME_ID
                                                );
        m_p_TF_broadcaster->sendTransform(tf_transform);
                                      

        /* Transform:  Camera -> Bitcraze */
        transform = tf::Transform(BCRAZE_TO_RSENSE_ROTATION, BCRAZE_TO_RSENSE_TRANSLATION);
        tf_transform = tf::StampedTransform(
                                              transform,
                                              ros::Time::now(),
                                              CAMERA_FRAME_ID,
                                              ODOMETER_FRAME_ID 
                                              );
        m_p_TF_broadcaster->sendTransform(tf_transform);

    }
    
    
    





    

//--------------------------------------------------------------------------
//                           ROS data generators
//--------------------------------------------------------------------------
    
    /*=======================================================
    * @brief           Get standard ROS odometry msg from snapshot
    *
    * @param           snapshot  -  Snapshot of the odometer data (see class definition)
    *
    * @return          ROS odometry msg, which can be published to a topic
    *
    * @author          Daniel Greenberger
    =========================================================*/
    nav_msgs::Odometry get_odom_to_lab_topic_msg(const OdometerData& snapshot)
    {
        
        /* Build ROS odometry msg */
        nav_msgs::Odometry odom;
        
        odom.header.stamp = snapshot.last_update_timestamp;
        
        odom.header.frame_id = LAB_FRAME_ID;
        odom.child_frame_id = ODOMETER_FRAME_ID;
        
        odom.pose.pose.position.x = snapshot.total_dist_x;
        odom.pose.pose.position.y = snapshot.total_dist_y;
        odom.pose.pose.position.z = snapshot.current_z_range;

        odom.twist.twist.linear.x = snapshot.current_speed_x;
        odom.twist.twist.linear.y = snapshot.current_speed_y;

        // TODO: add angular velocity
        
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(snapshot.total_rotation_z_axis);;
        
        return odom;
        
    }
    
    
    /*=======================================================
    * @description     Construct the current transformation between the RaceCar odometry frame and the lab,
    *                  according to the accumulated data from the odometer.
    * 
    * @param           snapshot  -  This is an a snapshot of odometry data which will be used to define the transformation 
    *                               
    * @return          Stamped transform to be published to tf 
    *
    * @author          Daniel Greenberger
    =========================================================*/
    geometry_msgs::TransformStamped get_odom_to_lab_TF_transform(const  OdometerData snapshot)
    {

        geometry_msgs::TransformStamped odom_trans;
        
        ros::Time curr_time = snapshot.last_update_timestamp;
        odom_trans.header.stamp = curr_time;
        
        odom_trans.header.frame_id = LAB_FRAME_ID;
        odom_trans.child_frame_id = ODOMETER_FRAME_ID;
        
        odom_trans.transform.translation.x = snapshot.total_dist_x;
        odom_trans.transform.translation.y = snapshot.total_dist_y;
        odom_trans.transform.translation.z = snapshot.current_z_range;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(snapshot.total_rotation_z_axis);

        return odom_trans;
    }

//--------------------------------------------------------------------------
//                           debug functions functions
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
//          m_p_TF_listener->transformPoint(CAMERA_FRAME_ID, odom_point, base_point);
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
