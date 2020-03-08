#ifndef _ROS_JESTON_DEFS_H_
#define _ROS_JESTON_DEFS_H_

/*
	This file is intended to contain all definitions and library includes
	related to the ROS API integration with the Jeston environment.
*/

#ifdef ROS_COMPILATION


///////////////////////
///    INCLUDES     ///
///////////////////////

/* Basic infrastructure */

#include "ros/ros.h" // includes all the headers necessary to use the most common public pieces of the ROS system.

/* Message types */
#include "std_msgs/String.h"





///////////////////////
///     MACROS      ///
///////////////////////

#define ROS_CODE_SECTION(code)      \
	do                              \
	{                               \
		code                        \
	}                               \
	while(0);

#define ROS_DEFINITION(text)    text





#else // No ROS integration

#define ROS_CODE_SECTION(code)


#define ROS_DEFINITION(text) 




#endif /* ROS_COMPILATION */




























#endif /*_ROS_JESTON_DEFS_H_*/