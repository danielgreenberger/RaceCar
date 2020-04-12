#ifndef _racecar_ccordinates_H_
#define _racecar_ccordinates_H_

#include "ros_defs.h"

/*

          RaceCar coordinate systems  (top view)


                       
			           Z+                                                                                                          
                       ^                                                                                                           
					   |     ____________                                                                                           
					   |    |            |                                                                                          
					   |    |     X+     |                                                                                          
					   |    | (Outwards) |                                                                                          
					   |    |____________|                                                                                          
		 ______________|_________________                                                                                          
		|								 |	                                                                                       
		|								 |                                                                                         
		|		    RealSense            | -----------------> Y+                                                                   
		|			(camera)			 |                                                                                         
		|________________________________|					                                                                                             
		|								 |	                                                                                       
		|								 |                                                                                         
		|		     JETSON              |                       X+ (BITCRAZE)  =>  Z+ (RealSense)                                          
		|		     (CPU)               |                       Y+ (BITCRAZE)  =>  Y+ (RealSense)                                          
		|		                         |                       Z+ (BITCRAZE)  =>  X- (RealSense)                                          
		|								 |                                                                                         
		|								 |                                                                                         
		|								 |                                                                                         
		|								 |                                                                                         
		|								 |                                                                                         
		|								 |                                                                                         
		|--------------------------------|                                                                  
		|								 |                                                                                         
		|								 |                                                                                         
		|								 |                                                                                         
		|								 |                                                                                         
		|								 |                                                                                         
		|								 |                                                                                         
		|								 |                                                                                         
		|								 |                                                                                         
		|								 |                                                                                         
		|________________________________|									                                                       
																															       
																		
		 ________________________________                                                                                          
		|								 |	                                                                                       
		|								 |                                                                                         
		|		     BITCRAZE            | -----------------> Y+                                                                   
		|		    (odometer)			 |                                                                                         
		|________________________________|									                                                       
					   |     ___________                                                                                          
					   |    |           |                                                                                         
					   |    |     Z+    |                                                                                         
					   |    | (Inwards) |                                                                                         
					   |    |___________|                                                                                         
		               |                                                                                                          
					   |                                                                                                          
					   |                                                                                                          
					   |                                                                                                          
					   V                                                                                                          
																														       
					   X-   (X is facing the direction of the car)                                                                                                     
					   
*/

//--------------------------------------------------------------------------
//                            RaceCar base frame
//--------------------------------------------------------------------------

// TODO: currently we define the base frame of the RaceCar to be the one of the RealSense camera
//       In case where another base is defined (i.e near the Jetson), we need to either:
//
//       1. Define the translation REALSENSE -> BASE, and keep the other translations intact
//          (They will still be evaluated correctly: OTHER_FRAME => REALSENSE => BASE) 
//
//       2. Define translations from all frames (i.e RealSense, Bitcraze) to Base.
//
//       The 2 approaches may differ in efficiency, so it needs to be cheched how ROS TF stores and 
//       calculates the transformations.

const std::string BASE_FRAME_ID = "racecar_base_frame";






/*
        Transformation:  REALSENSE => BASE  (Identity transformation)
*/

ROS_DEFINITION(

                                          
            const auto RSENSE_TO_BASE_ROTATION = tf::Matrix3x3::getIdentity();
                                                                  	
            const auto RSENSE_TO_BASE_TRANSLATION = tf::Vector3(
                                                                  0.0,    /* Distance in RealSense POV - x */
                                                                  0.0,    /* Distance in RealSense POV - y */
                                                                  0.0     /* Distance in RealSense POV - z */
                                                               );
                                          
)



//--------------------------------------------------------------------------
//                            RealSense (camera) frame
//--------------------------------------------------------------------------

const std::string CAMERA_FRAME_ID = "racecar_camera_frame";



/*
        Transformation:  BITCRAZE => REALSENSE
*/


ROS_DEFINITION(


                                          
          const auto BCRAZE_TO_RSENSE_ROTATION = tf::Matrix3x3    (
                                                                     0,    0,   -1, 
                                                                     0,    1,    0,
                                                                     1,    0,    0
                                                                  );	



                                                                  
          const auto BCRAZE_TO_RSENSE_TRANSLATION = tf::Vector3(
                                                                  0.0,    /* Distance in RealSense POV - x */
                                                                  0.0,    /* Distance in RealSense POV - y */
                                                                 -0.5     /* Distance in RealSense POV - z */
                                                               );
                                          
)

//--------------------------------------------------------------------------
//                            Bitcraze (odometer) frame
//--------------------------------------------------------------------------

const std::string ODOMETER_FRAME_ID = "racecar_odometer_frame";








#endif  /* _racecar_ccordinates_H_ */                      
