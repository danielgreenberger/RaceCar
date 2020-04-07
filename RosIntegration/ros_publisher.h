#ifndef _ROS_JETSON_PUBLISHER_H_
#define _ROS_JETSON_PUBLISHER_H_

/*
	@DESCRIPTION
	
	This is a template class for a ROS publisher thread. 
	The publisher is intergated to the main, non-ROS-native program as a class member. 
	The publisher object is comprised of 2 main components:
		1. A FIFO memory buffer.
		2. A thread which registers on the ROS system and publishes the FIFO content to a given ROS topic.  
	
	The implementation using a buffer is done is order to minimize the latency on the RaceCar 
    main thread. 
    
    The drawback is an increased latency in publishing a single message to ROS, since it has to go through the FIFO. 
*/

#ifdef ROS_COMPILATION

#include <string>
#include <queue>
#include <sstream>
#include <iostream>
#include <thread>
#include "ros_defs.h"
#include "basic_utils.h"


namespace RosIntegration
{
using string = std::string;

////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                           MACROS                                             /////
////////////////////////////////////////////////////////////////////////////////////////////////////////


/* Debug printing */
#define ROS_INTEGRATION_DEBUG_PRINT_ENABLE (0)  // 0 - Debug printing is OFF, 1 - Debug printing is ON

#if (ROS_INTEGRATION_DEBUG_PRINT_ENABLE == 1)
#define ROS_INTEGRATION_DEBUG_PRINT(text)       \
	do                                          \
	{                                           \
		std::cout << text << std::endl;         \
	} while(0);                                 
	
#else
#define ROS_INTEGRATION_DEBUG_PRINT(text)  
#endif




/*
         Critical section macro. 
         Must only be used inside the Publisher class.
*/
#define ROS_PUBLISHER_CRITICAL_SECTION(code)   do{  __CRITICAL_SECTION(m_publisher_mutex, code)  } while(0);     




////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                  PUBLISHER  CONFIGURATION                                    /////
////////////////////////////////////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------
//                           Default max FIFO capacity
//--------------------------------------------------------------------------
/*      
        This defines an upper limit on the FIFO capacity to store messages, before sending. 
        Since the FIFO memory is dynamically allocated, a limit is needed in order to avoid 
        the flooding of the FIFO and running out of memory, which can be dangetour for the 
        other RaceCar flows.
*/
const int ROS_TOPIC_QUEUE_BUFFER_SIZE = 1000;



//--------------------------------------------------------------------------
//                           Default polling rate 
//--------------------------------------------------------------------------
/* 
        This is the rate in which the FIFO will be checked for new messages
        to be published to the ROS system.
*/
const int ROS_PUBLISH_RATE_PER_SECOND = 10;





//--------------------------------------------------------------------------
//                            Optional flags for publisher class
//--------------------------------------------------------------------------
/*
       IMPORTANT: The number of flag may not exceed 32, as flags are represented as a 32-bit mask. 
*/

typedef enum : uint32_t
{
    /*
        If set, the Publisher will not start automatically upon the object construction, 
        and will require to be started explicitly.
    */
    OPTION_FLAG_NO_START_ON_INIT             = __BIT(0), 
    
    /*
        If set, the name of the object node in the ROS system will be determined by the user. 
    */
    OPTION_FLAG_DEFINE_NODE_NAME             = __BIT(1), 
    
    
    /*
        These flags control the behavior of the FIFO when trying to push a message after reaching the max pre-defined
        capacity. 
        
        The default behavior is to drop all messages arriving to the FIFO when it's already full.
        
        The flags can change the default behavior:
        OPTION_FLAG_ASSERT_ON_MAX_CAPACITY - if set, throw an exception (can be used for debug).
        OPTION_FLAG_WARN_ON_MAX_CAPACITY   - if set, a warning message will be printed.
        OPTION_FLAG_OVERRIDE_MAX_CAPACITY  - If set, no messages will be ignored (not recommended for real-time applications).
        
        ASSERT and OVERRIDE flags are mutually exclusive, and can't be both set. 
    */   
    OPTION_FLAG_ASSERT_ON_MAX_CAPACITY       = __BIT(2),  
    OPTION_FLAG_WARN_ON_MAX_CAPACITY         = __BIT(3),  
    OPTION_FLAG_OVERRIDE_MAX_CAPACITY        = __BIT(4),  
    
    

    OPTION_FLAG_MAX                          = __BIT(31), 
   // TODO: implement OPTION_FLAG_DUMP_TO_FILE? Can also use rosbag for output.
} option_flags_e;


const int DEFAULT_FLAGS = OPTION_FLAG_WARN_ON_MAX_CAPACITY;




//--------------------------------------------------------------------------
//                              ROS node name
//--------------------------------------------------------------------------

const string INVALID_NODE_NAME = "";
const string DEFAULT_NAME_PREFIX = "Jetson_Publisher_";







////////////////////////////////////////////////////////////////////////////////////////////////
/////                                  PUBLISHER  CLASS                                    /////
////////////////////////////////////////////////////////////////////////////////////////////////


template <typename T>
class Publisher
{
    
private:
    
    // FIFO
    uint32_t m_max_queue_size;
    std::queue<T> m_queue;
    
    
    // Mutex
    std::mutex m_publisher_mutex;
    
    // Flags
    uint32_t m_flags;
    
    // Thread
    std::thread m_ros_node_thread; 
    bool m_publisher_enabled; 
    
    // ROS-related
    string m_node_name;
    const string m_topic_name;

    

//--------------------------------------------------------------------------
//                           Publisher interface functions
//--------------------------------------------------------------------------
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
    Publisher (string topic_name, uint32_t flags = DEFAULT_FLAGS, uint32_t max_queue_size = ROS_TOPIC_QUEUE_BUFFER_SIZE)
        :
        Publisher (topic_name, INVALID_NODE_NAME, flags, max_queue_size)
    {
        
    }
    
    /*=======================================================
    * @brief           Constructor for the publisher class allowing to define a custom name for the node.
    *
    * @param           topic_name       -  Name of the topic for which the messages will be published.
    *
    * @param           node_name       -   Name of the publisher node in the ROS system.
    *
    * @param           flags            - (Optional) configuration flags for publisher class.
    *
    * @param           max_queue_size   - (Optional) Maximum number of outgoing messages to be queued for delivery to subscribers.
    *
    * @author          Daniel Greenberger
    =========================================================*/
    Publisher (string topic_name, string node_name,  
                uint32_t flags = DEFAULT_FLAGS, uint32_t max_queue_size = ROS_TOPIC_QUEUE_BUFFER_SIZE)
        :
        m_topic_name(topic_name),
        m_node_name(), 
        m_max_queue_size(max_queue_size),
        m_queue(), 
        m_publisher_mutex(), 
        m_flags(flags), 
        m_ros_node_thread(), 
        m_publisher_enabled(false) 
    {
        /* Check option flags */
        assert_flag_mask_validity();
        
        
        /* Set node name */
        bool use_default_name = !is_flag_set(OPTION_FLAG_DEFINE_NODE_NAME);
        
        if (use_default_name) // TODO: bug, we need to remove "/" from name
        {
            m_node_name = DEFAULT_NAME_PREFIX + m_topic_name;
        }
        else
        {
            ASSERT(node_name.size() > 0, std::exception()); // TODO: make custon exception
            m_node_name = node_name;
        }
        
        
        /* Start Publisher */
        if (!is_flag_set(OPTION_FLAG_NO_START_ON_INIT))
        {
            StartPublisher();
        }

        
    }
    
    // 
    
    /*=======================================================
    * @brief           Publish a message to the ROS system. 
    *
    * @description     This method allows the caller to published a message to the ROS system 
    *                  on the pre-defined topic of this publisher object. 
    *                  
    *                  The message will be copied into the publisher's FIFO, allowing the 
    *                  calling thread to free its local copy of the message.
    *
    * @pre             The publisher thread must be enabled (one must call StartPublisher() ).
    *
    * @param           msg  -  The message to be published.
    *
    * @return          None
    *
    * @author          Daniel Greenberger.
    =========================================================*/
    void Publish(T& msg) 
    {
        ASSERT(m_publisher_enabled, std::exception());
        ROS_INTEGRATION_DEBUG_PRINT("publish :: pushing message to back of the queue");
        
        bool queue_not_full;
       /* Push message to FIFO */
        ROS_PUBLISHER_CRITICAL_SECTION
        (
            const uint32_t queue_size = m_queue.size();
            queue_not_full = (queue_size <= m_max_queue_size);
            if ( queue_not_full || (is_flag_set(OPTION_FLAG_OVERRIDE_MAX_CAPACITY))  )
            {
                m_queue.push(msg);
            }
        )
            
            
        /* Handle full FIFO case */
        if (is_flag_set(OPTION_FLAG_ASSERT_ON_MAX_CAPACITY))
        {
            ASSERT(queue_not_full, std::exception());
        }
        if ( !queue_not_full && (is_flag_set(OPTION_FLAG_WARN_ON_MAX_CAPACITY))  )
        {
            ROS_WARN_STREAM("Publisher :: FIFO buffer is full, messages may be dropped.  Node: " << m_node_name);
        }
    }
    
    
    
    /*=======================================================
    * @brief           Desctructor for the publisher object.
    *
    * @param           None.
    *
    * @return          None.
    *
    * @author          Daniel Greenberger
    =========================================================*/
    ~Publisher()
    {
        HaltPublisher();
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
    void StartPublisher()
    {
        ASSERT(false == m_publisher_enabled, std::exception()); // TODO: add exception
        
        ROS_INTEGRATION_DEBUG_PRINT(" StartPublisher :: Starting ROS publisher thread ");
        m_ros_node_thread = std::thread(&RosIntegration::Publisher<T>::__node_main, this);
        m_publisher_enabled = true;
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
    void HaltPublisher()
    {
        // Check if publisher is already off
        if (!m_publisher_enabled)
        {
            ROS_INTEGRATION_DEBUG_PRINT("HaltPublisher :: Publisher is already off!");
            return;
        }
        
        // Mark the thread to stop execution
        m_publisher_enabled = false;
        
        // Wait for the publisher to finish processing current message
        ROS_INTEGRATION_DEBUG_PRINT("HaltPublisher :: waiting for thread.");
        m_ros_node_thread.join();
        ROS_INTEGRATION_DEBUG_PRINT("HaltPublisher :: Thread has halted.");
    }
    



//--------------------------------------------------------------------------
//                             Misc functions
//--------------------------------------------------------------------------
private:    
    
    /*=======================================================
    * @brief           Check if the given option flag is set.
    *
    * @param           flag  -  Option flag to be checked if set.
    *
    * @return          TRUE  - The option flag is set for the object.
    *                  FALSE - The option flag is not set for the object.
    *
    * @author          Daniel Greenberger
    =========================================================*/
    bool is_flag_set(option_flags_e flag)
    {
        return (m_flags & flag);
    }
    
    
    /*=======================================================
    * @brief           Check that the flag mask is valid
    *
    * @return          TRUE  - The flag mask is valid.
    *                  FALSE - The flag mask is invalid.
    *
    * @author          Daniel Greenberger
    =========================================================*/
    void assert_flag_mask_validity()
    {
        /* Max capacity */
        const bool flag_override_set = is_flag_set(OPTION_FLAG_OVERRIDE_MAX_CAPACITY);
        const bool flag_assert_set = is_flag_set(OPTION_FLAG_ASSERT_ON_MAX_CAPACITY);
        const bool both_flags_set = flag_override_set && flag_assert_set;
        ASSERT(both_flags_set == false, std::exception());
        
        
    }
    
    

    

//--------------------------------------------------------------------------
//                           Publisher node functions
//--------------------------------------------------------------------------
    
private:    
    
    
    
    /*
        TODO: Make sure that max capacity is enforced somewhere (maybe in the queue itself) or remove from API.
        
    */

    
    
    /*=======================================================
    * @brief           Publisher node main function
    *
    * @description     This function is where the object interfaces the ROS system. 
    *                  We are checking the internal object FIFO for new messages. 
    *                  When a new message exists, it is published to the ROS topic
    *                  which was pre-defined for this object.
    *
    *                  NOTE:
    *                  While the publisher is enabled, the function works in an infinate loop 
    *                  and never returns. Therefore, it must be executed on a new thread to not
    *                  block RaceCar's execution. 
    *
    * @param           None.
    *
    * @return          None.
    *
    * @author          Daniel Greenberger
    =========================================================*/
    void __node_main()
    {
        /* Init ROS node */
        int node_init_arg_counter = 0;
        ros::init(node_init_arg_counter, nullptr, m_node_name);
        ASSERT(ros::master::check(), std::exception()); // Make sure ROS main process is running
        ros::NodeHandle node_handler; 

        
        
        /* Register to publish for topic */
        ros::Publisher node_publisher = node_handler.advertise<T>(m_topic_name, m_max_queue_size);

        
        
        /* Define publish frequency */
        ros::Rate loop_rate(ROS_PUBLISH_RATE_PER_SECOND);

        
        
        /* Publish */
        int count = 0;
        while (_Usually(ros::ok && m_publisher_enabled))
        {
            
            // Pull a message from FIFO
            T msg;
            bool msg_exists = pull_message_if_exists(msg);
            
                
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


        
        /* Finish node execution */
        ROS_INFO_STREAM("ROS publisher stop.  Node name: " << m_node_name); 
        ROS_INTEGRATION_DEBUG_PRINT("ROS publisher stop. ");
        m_publisher_enabled = false;
        
    }
    
    
    
    
    /*=======================================================
    * @brief           Pull a message from the FIFO
    *
    * @description     This function is performing an access to the queue in critical section. 
    *                  If a new message exists, it is pulled from the queue and returned to the caller
    *                  using the output argument. 
    *                  
    * @param[out]      msg_output  -  Message object provided by the callee to contain the new message (if exists)
    *
    * @return          TRUE   -  A new message has been pulled from the FIFO.
    *                  FALSE  -  No new messages exist in the FIFO.
    *
    * @author          Daniel Greenberger
    =========================================================*/
    bool pull_message_if_exists(T& msg_output)
    {
        bool msg_exists = false;
        
        ROS_PUBLISHER_CRITICAL_SECTION
        (
            msg_exists = (false == m_queue.empty());

            if (msg_exists)
            {    
                msg_output = m_queue.front();  
                m_queue.pop();
            }
        )
        
        return msg_exists;
    }

    
    
};

#undef ROS_PUBLISHER_CRITICAL_SECTION


}




#endif /* ROS_COMPILATION */


#endif /*_ROS_JETSON_PUBLISHER_H_*/
