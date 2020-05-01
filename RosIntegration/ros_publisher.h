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
    
    The underlying container used for the FIFO is vector (using a deque interface), rather than a list. 
    This is in order to use the spatial and temporal locality nature of the data access, as messages are published in a 
    consecutive order.
    
    
    The drawback is an increased latency in publishing a single message to ROS, since it has to go through the FIFO. 

    TODOs:
    1. Extend to "RosInterface" and make Publisher a derived class, allowing to derive listener as well. 
    2. Add configuration for ROS buffering. 
    3. Add custom exceptions indtead of using std::exception.
    
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
#define ROS_INTEGRATION_DEBUG_PRINT_ENABLE (0)  // 0 - Debug printing is OFF, 
                                                // 1 - Debug printing is ON



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
         
         A critical section is needed here, since the underlying container used for the FIFO is vector (using deque interface).
         According to the standard, one needs to assume that any (addition / removal) of items may cause a mem-copy of the entire 
         data which will invalidate all the existing iterators. 
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
        the flooding of the FIFO and running out of memory, which can be dangerous for the 
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
        
        ASSERT and OVERRIDE flags are mutually exclusive, and therefore can't be both set. 
    */   
    OPTION_FLAG_ASSERT_ON_MAX_CAPACITY       = __BIT(2),  
    OPTION_FLAG_WARN_ON_MAX_CAPACITY         = __BIT(3),  
    OPTION_FLAG_OVERRIDE_MAX_CAPACITY        = __BIT(4),  

// TODO: other flags to maybe implement:
    // FLAG_PUBLISH_BUFFER_MSGS_ON_EXIT
    // RESERVE_MAX_BUFFER_SIZE
        
    OPTION_FLAG_MAX                          = __BIT(31), 
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
    std::mutex m_publisher_mutex; // See comment at critical-section macro about the need of the mutex.
    
    // Flags
    uint32_t m_flags;
    
    // Thread
    std::thread m_ros_node_thread; 
    bool m_is_active; 
    
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
        m_is_active(false) 
    {
        /* Check option flags */
        assert_flag_mask_validity();
        
        
        /* Set node name */
        bool use_default_name = !is_flag_set(OPTION_FLAG_DEFINE_NODE_NAME);
        
        if (use_default_name) 
        {
            m_node_name = DEFAULT_NAME_PREFIX + m_topic_name;
        }
        else
        {
            ASSERT(node_name.size() > 0, std::exception()); 
            m_node_name = node_name;
        }
        
        
        /* Start Publisher */
        if (!is_flag_set(OPTION_FLAG_NO_START_ON_INIT))
        {
            start();
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
    * @pre             The publisher thread must be enabled (one must call start() ).
    *
    * @param           msg  -  The message to be published.
    *
    * @return          None
    *
    * @author          Daniel Greenberger.
    =========================================================*/
    void publish(T& msg) 
    {
        ASSERT(m_is_active, std::exception());
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
        stop();
    }

    
    
    /*=======================================================
    * @brief           Start the publisher
    *
    * @description     This function starts a separate thread which publishes available messages from the FIFO 
    *                  to the ROS topic defined at initialization.
    *
    * @param           None.
    *
    * @return          None.
    *
    * @author          Daniel Greenberger
    =========================================================*/
    void start()
    {
        ASSERT(false == m_is_active, std::exception());  
        
        ROS_INTEGRATION_DEBUG_PRINT(" start :: Starting ROS publisher thread ");
        m_is_active = true;
        m_ros_node_thread = std::thread(&RosIntegration::Publisher<T>::__node_main, this);
    }
    
    
    /*=======================================================
    * @brief           Stop the publisher
    *
    * @description     This function stops the publisher thread. 
    *                  Any messages remaining on the FIFO will be discarded.
    *
    * @param           None.
    *
    * @return          None.
    *
    * @author          Daniel Greenberger
    =========================================================*/
    void stop()
    {
        /* Check if publisher is already off */
        if (!m_is_active)
        {
            ROS_INTEGRATION_DEBUG_PRINT("stop :: Publisher is already off!");
            return;
        }
        
        
        /* Mark the thread to stop execution */
        m_is_active = false;
        
        
        /* Wait for the publisher to finish processing current message */
        ROS_INTEGRATION_DEBUG_PRINT("stop :: waiting for thread.");
        m_ros_node_thread.join();
        ROS_INTEGRATION_DEBUG_PRINT("stop :: Thread has halted.");
    }
    



//--------------------------------------------------------------------------
//                             Option flags functions
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
    bool is_flag_set(const option_flags_e flag)
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
//                           Publisher node infrastructure
//--------------------------------------------------------------------------
    
private:    
      
    
    /*=======================================================
    * @brief           Publisher node main function
    *
    * @description     This function is where the object interfaces the ROS system, 
    *                  and is a wrapper for all the internal publisher logic within ROS, 
    *                  which may be overriden by inheritance. 
    *                  
    *                  This is where the publisher is registers as a new node in ROS.
    *                  The node is destroyed when the function returns.
    *
    *                  NOTE:
    *                  While the publisher is enabled, the function calls an infinate loop 
    *                  for the internal publisher. Therefore, it must be executed on a new thread to not
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


        /* Enter main publisher loop */
        __publisher_loop(node_handler);

        
        /* Finish node execution */
        ROS_INFO_STREAM("ROS publisher stop.  Node name: " << m_node_name); 
        ROS_INTEGRATION_DEBUG_PRINT("ROS publisher stop. ");
        m_is_active = false;
        
    }
    
protected:    
    /*=======================================================
    * @brief           Check if the publisher is active
    *
    * @description     This function is intended to be used by the publisher main
    *                  loop for its halting condition. 
    *
    * @return          TRUE   -  The publisher is active
    *                  FALSE  -  The publisher is not active
    *
    * @author          Daniel Greenberger
    =========================================================*/
    bool active() 
    {
        return _Usually(m_is_active && ros::ok );
    }
    
    
    
    /*=======================================================
    * @brief           Get the ROS topic of this publisher
    =========================================================*/
    const std::string get_topic_name()
    {
        return m_topic_name;
    }

    




//--------------------------------------------------------------------------
//                              FIFO   functions 
//--------------------------------------------------------------------------
    
protected:    
    
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

    

//---------------------------------------------------------------------------------------------
//                           Internal Publisher functions (can be overriden by derived class)
//---------------------------------------------------------------------------------------------
        
        
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
    virtual void __publisher_loop(ros::NodeHandle& node_handler)
    {
        
        /* Register to publish for topic */
        ros::Publisher node_publisher = node_handler.advertise<T>(m_topic_name, m_max_queue_size);

        
        /* Define publish frequency */
        ros::Rate loop_rate(ROS_PUBLISH_RATE_PER_SECOND);

        
        /* Publish */
        int count = 0;
        while ( active() )
        {
            
            T msg;
            bool msg_exists = pull_message_if_exists(msg);
            
            if (msg_exists)
            {				
                ROS_INTEGRATION_DEBUG_PRINT("Publishing message: " << ++count);  
                node_publisher.publish(msg);
                ros::spinOnce();
            }

            loop_rate.sleep();
        }
        
    }


    
    
};

#undef ROS_PUBLISHER_CRITICAL_SECTION


}




#endif /* ROS_COMPILATION */


#endif /*_ROS_JETSON_PUBLISHER_H_*/
