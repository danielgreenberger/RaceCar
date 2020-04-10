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
        tf::TransformBroadcaster broadcaster;

        
        
        /* Define publish frequency */
        ros::Rate loop_rate(ROS_PUBLISH_RATE_PER_SECOND);

        
        
        /* Publish */
        int count = 0;
        while ( this->active() )
        {
            
            // Generate transform
            
         //   auto rotation = tf::Quaternion(0, 0, 0, 1);
            auto translation_vector = tf::Vector3(
                                            0.0,    /* x */
                                            0.0,    /* y */
                                            0.0     /* z */
                                          );
                                          
            auto rotation_matrix = tf::Matrix3x3    (
                                                        1,   0,   0,
                                                        0,   1,   0,
                                                        0,   0,   1 
                                                    );	
                                          
                                          
                                          
            auto transform = tf::Transform(rotation_matrix, translation_vector);
            
            auto tf_transform = tf::StampedTransform(
                                                    transform,
                                                    ros::Time::now(),
                                                    "base_link", 
                                                    "base_laser"
                                                    );
                
            
            broadcaster.sendTransform(tf_transform);

                
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
            loop_rate.sleep();
            
        }
        
    }
  
};

#undef ROS_PUBLISHER_CRITICAL_SECTION

//    ros::Rate r(100);
//   
//    tf::TransformBroadcaster broadcaster;
//   

}




#endif /* ROS_COMPILATION */


#endif /*_ROS_JETSON_TF_PUBLISHER_H_*/
