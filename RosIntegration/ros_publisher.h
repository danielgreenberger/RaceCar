#ifndef _ROS_JETSON_PUBLISHER_H_
#define _ROS_JETSON_PUBLISHER_H_

/*
	@DESCRIPTION
	
	This is a template class for a ROS publisher thread. 
	The publisher is intergated to the main, non-ROS-native program as a class member. 
	The publisher object is comprised of 2 main components:
		1. A FIFO memory buffer.
		2. A thread which registers on the ROS system and publishes the FIFO content to a given ROS topic.  
	
	TODO add design considerations and trade-offs.
*/

#ifdef ROS_COMPILATION

#include <string>
#include <queue>
#include <sstream>
#include <iostream>
#include <thread>
#include "ros_defs.h"




/////////////////////////////////////
/////           MACROS          /////
/////////////////////////////////////

// Debug printing
#define ROS_INTEGRATION_DEBUG_PRINT_ENABLE 0

#if (ROS_INTEGRATION_DEBUG_PRINT_ENABLE == 1)
#define ROS_INTEGRATION_DEBUG_PRINT(text)       \
	do                                          \
	{                                           \
		std::cout << text << std::endl;         \
	} while(0);                                 
	
#else
#define ROS_INTEGRATION_DEBUG_PRINT(text)  
#endif




// This must be called from within the Publisher class only
#define CRITICAL_SECTION(code)       		  \
	do                               		  \
	{                                		  \
		enter_critical_section();    		  \
		code                         		  \
		exit_critical_section();     		  \
	} while(0);



/////////////////////////////////////
/////           CLASS           /////
/////////////////////////////////////

namespace RosIntegration
{
	
	using std::string;

	constexpr int ROS_TOPIC_QUEUE_BUFFER_SIZE = 1000;
	constexpr int ROS_PUBLISH_RATE_PER_SECOND = 10;
	typedef unsigned long int uint_32;
	
	constexpr uint_32 __BIT(uint_32 i)
	{
		return (1UL << i);
	}
	
	// TODO use enum class and define implicit cast?
	constexpr int NO_FLAGS = 0;
	typedef enum
	{
		OPTION_FLAG_NO_START_ON_INIT             = __BIT(0), 
		OPTION_FLAG_DUMP_TO_FILE                 = __BIT(1), 
		OPTION_FLAG_DEFINE_NODE_NAME             = __BIT(2), 
	} option_flags_e;
	
	


	template <typename T>
	class Publisher
	{
	public:
		Publisher (string topic_name, string node_name = "",  uint_32 flags = NO_FLAGS, uint_32 max_queue_size = ROS_TOPIC_QUEUE_BUFFER_SIZE)
			:
                        m_topic_name(topic_name),
			m_node_name(node_name), 
			m_max_queue_size(),
			m_queue(), 
			m_mutex(), 
			m_flags(flags), 
			m_ros_thread(), 
			m_publisher_enabled(false), 
			m_argc(0)
		{
			if (!is_flag_set(OPTION_FLAG_DEFINE_NODE_NAME)) // dgreenbe todo bug, we need to remove "/" from name
			{
				m_node_name = "Jetson_Publisher_" + m_topic_name;
			}
			
			// Start Publisher
			if (!is_flag_set(OPTION_FLAG_NO_START_ON_INIT))
			{
				StartPublisher();
			}

			
		}
		
		// Publish to node
		void Publish(T& msg) // TODO use a reference or copy?
		{
			if (!m_publisher_enabled)
			{
				ROS_INTEGRATION_DEBUG_PRINT("publish :: ROS node is not working! ");
				return;
			}
			
			CRITICAL_SECTION
			(
				m_queue.push(msg);
				ROS_INTEGRATION_DEBUG_PRINT("publish :: pushing message to back of the queue");
			)
		}
		
		
		// TODO destructor, needs to call stopPublisher
		~Publisher()
		{
			HaltPublisher();
		}

		
		// Thread-related
		void StartPublisher()
		{
			if (!m_publisher_enabled)
			{
				m_publisher_enabled = true;
				ROS_INTEGRATION_DEBUG_PRINT(" StartPublisher :: Starting ROS publisher thread ");
				m_ros_thread = std::thread(&RosIntegration::Publisher<T>::init_ros_publisher_node, this);
			}
			else
			{
				ROS_INTEGRATION_DEBUG_PRINT(" StartPublisher ::  m_publisher_enabled is already TRUE");
			}

		}
		
		
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
			m_ros_thread.join();
			ROS_INTEGRATION_DEBUG_PRINT("HaltPublisher :: Thread has halted.");
		}
		
	private:

		////////////////////////
		///      Members     ///
		////////////////////////
		
		// FIFO
		uint_32 m_max_queue_size;
		std::queue<T> m_queue;
		
		
		// Mutex
		std::mutex m_mutex;
		uint_32 m_flags;
		
		// Thread
		std::thread m_ros_thread; // TODO think of better name?
		bool m_publisher_enabled; // TODO explain this is used as a flag
		
		// ROS-related
		string m_node_name;
		const string m_topic_name;
		int m_argc;
		
		
		
		////////////////////////
		///      Methods     ///
		////////////////////////
		
		// Getters/Setters 
		int get_max_buffer_capacity();
		void set_max_buffer_capacity(int);
		
		
		// Flags
		bool is_flag_set(option_flags_e flag)
		{
			return (m_flags & flag);
		}
		
		
		
		
		void enter_critical_section()
		{
			m_mutex.lock();
			ROS_INTEGRATION_DEBUG_PRINT(" Entered critical section ");
		}
		
		
		void exit_critical_section()
		{
			m_mutex.unlock();
			ROS_INTEGRATION_DEBUG_PRINT(" Left critical section ");
		}
		
		
		/*
			TODO:
			1. Extend static message to a queue, 
			2. Make sure to add mutex
			4. Insert function and queue into a class
			3. Extend to template
		*/

		// TODO maybe think of a better name?
		
		void init_ros_publisher_node()
		{
			/* Init ROS node */
			ros::init(m_argc, nullptr, m_node_name);
			if (false == ros::master::check())
			{
				exit(0); // TODO maybe just return or print an error?
			}
			ros::NodeHandle node_handler; 
			
			/* Register to publish for topic */
			ros::Publisher node_publisher = node_handler.advertise<T>(m_topic_name, ROS_TOPIC_QUEUE_BUFFER_SIZE);
			
			/* Define publish frequency */
			ros::Rate loop_rate(ROS_PUBLISH_RATE_PER_SECOND);
			
			/* Publish */
			int count = 0;
			while (ros::ok && m_publisher_enabled)
			{
				
				// Pull a message from FIFO
				bool msg_exists = false;
				T msg;
				CRITICAL_SECTION
				(
					msg_exists = (false == m_queue.empty());
					if (msg_exists)
					{
                                                
						msg = m_queue.front();  // TODO: use memcopy, reference, or pointer?
						m_queue.pop();
					}
				)
				
				// Publish
				if (msg_exists)
				{				
					// ROS debug print
					ROS_INFO("Publishing message: %i", ++count);
					
					// Publish
					node_publisher.publish(msg);
					ros::spinOnce();
				}
				
				// Wait for next publish interval 
				loop_rate.sleep();
				
			}
			
			/* Finish node execution */
			ROS_INFO("ROS publisher stop. "); // TODO add string with node name, also maybe add print for main program?
			ROS_INTEGRATION_DEBUG_PRINT("ROS publisher stop. ");
                        m_publisher_enabled = false;
			
		}
		
		
	};

#undef CRITICAL_SECTION


}




#endif /* ROS_COMPILATION */


#endif /*_ROS_JETSON_PUBLISHER_H_*/
