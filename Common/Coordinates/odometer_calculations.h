#ifndef BCRZ_LINEAR_TO_ROT
#define BCRZ_LINEAR_TO_ROT

#include <cmath>
#include <utility>
#include <tuple>
#include "bitcraze_types.h"

namespace OdometerTransform
{ 

//----------------------------------------------------------------------------------------------------------------------
//                                             Rotational transforms
//----------------------------------------------------------------------------------------------------------------------
/*
					
					Calculation:  Rotation by t from Bitcraze linear data
					

                     X___________                               					 
                     |   (Dy)   /                      
					 |         /           
					 |        /            
					 |       /             
					 |      /              
			    (Dx) |     / (Dr)             
					 |    /                
					 |(t)/                 
					 |  /                  
					 | /                   
					 |/_ _ _ _ _ _ _ _ _ _ _ _ _ Y                   
					 

-----------  Calculation of t ----------------
			
			t is the angle of rotation.
			
		    (Dx)/(Dr) = cos(t)
		    (Dy)/(Dr) = sin(t)	
			
		
		    For Dy = 0:  
            =>   t = 0  since  Dr = Dx 
		   
     	    For Dy != 0:
			=>	(Dx)/(Dy) = cos(t) / sin(t)  = cot(t)	
			=> 	t = arccot (Dx/Dy)
			=>  t = PI/2 - arctan(Dx/Dy)     (Since for every angle w ->  arccot(w) = PI/2 - arctan(w):)
			

*/

inline double get_angle_from_linear_offset(const double Dx, const double Dy)
{
    double angular_movement_radians;
    
    if (Dy == 0)
    {
        angular_movement_radians = 0;
    }
    else
    {
        angular_movement_radians = M_PI/2 - atan(Dx/Dy);
    }
    
    return angular_movement_radians;
}








/*
					
					Calculation:  Convert Dx'  Dy'  from current odometer frame to Dx Dy, which are the X-Y offset in the lab frame 
					

                     X           X'        					 
                     |          /            
					 |  ->     / 
					 |        /  
					 |       /   
					 |      /    
			    (Dx) |     /        
					 |    /      
					 | t /       
					 |  /        
					 | /  90-t       
					 |/_ _ _ _ _ _ _ _ _ _ _ _ _ Y                   
                     |\  t                                      					 
                     | \                                       					 
                     |  \                                      					 
                     |   \                                     					 
                     |90-t\
                     |     \
                     |      \
                     |       \
                     |      t \
                     |_ _ _ _ _\
                                Y'   (Perpendicular to X')                                   					 
                      

        According to the diagram above:
                                                              					 
        X = X' * cos(t)  -  Y' * sin(t)
`       Y = X' * sin(t)  +  Y' * cos(t)        
                                                              					 
                                                              					 
                                                              					 
                                                              					 

*/



/*=======================================================
* @brief           Converts distance travelled from   X'-Y'  to  X-Y  frame.
*
* @description     See the diagram above which justifies the calculation. 
*
* @param           t  -  The angle (radians) by which X' and Y'  are rotated  
*
* @param           r_Dx   -  The distance in X' units
*
* @param           r_Dy   -  The distance in Y' units
*
* @return          A pair (Dx,Dy)  representing the difference in X and Y  (lab) frame.
*
* @author          Daniel Greenberger
=========================================================*/
inline std::pair<double,double> rotate_by_angle (const double t, const double r_Dx, const double r_Dy)
{
    const double dx = r_Dx*cos(t) - r_Dy*sin(t);
    const double dy = r_Dx*sin(t) + r_Dy*cos(t);
    
    return std::make_pair(dx,dy);
}





//----------------------------------------------------------------------------------------------------------------------
//                                       Velocity and Distance cauculations
//----------------------------------------------------------------------------------------------------------------------

struct OdometerDataUnit
{
    double dx__meters;  /* Distance travelled (meters) in the X direction */
    double dy__meters;  /* Distance travelled (meters) in the Y direction */
    
    double vx__meters_p_sec;  /* Speed in the X direction (m/s) */
    double vy__meters_p_sec;  /* Speed in the Y direction (m/s) */
    
    double z_range__meters;    /* Current height of the sensor from the ground in meters */
    
    double dt__sec;   /* Time elapsed (in seconds) since the last measurement - this is the duration it took for dx,dy to form */
};


/* Conversion constants */
const double BITCRAZE_FOV_DEGREES = 42;
const double DEGREES_PER_PI = 180;
const double BITCRAZE_FOV_RADIANS = (BITCRAZE_FOV_DEGREES / DEGREES_PER_PI) * M_PI;
const double MILIMETERS_PER_PIXEL = 30;


/*=======================================================
* @brief           Metric unit conversions
=========================================================*/

inline double MILIMETER_TO_METER(const double x_mm)
{
    return 10e-3 * x_mm;
}

inline double MILISECOND_TO_SECOND(const double x_ms)
{
    return 10e-3 * x_ms;
}



/*=======================================================
* @brief           Convert Bitcraze data to metric units
*
* @param           bitcraze_input  -  Bitcraze odometer reading.
*
* @return          Odometer data in metric format.
*
* @author          Daniel Greenberger
=========================================================*/
inline OdometerDataUnit convert_to_metric_units(const Flow& bitcraze_input)
{
    const double dx_pixels = static_cast<double>(bitcraze_input.deltaX);
    const double dy_pixels = static_cast<double>(bitcraze_input.deltaY);
    const double z_height_mm = static_cast<double>(bitcraze_input.range);
    const double dt_ms = static_cast<double>(bitcraze_input.dt);

    // Note: the formula is taken from the AutoRaceCar document (page 20)
    const double dx_mm = z_height_mm * BITCRAZE_FOV_RADIANS * dx_pixels / (MILIMETERS_PER_PIXEL);
    const double dy_mm = z_height_mm * BITCRAZE_FOV_RADIANS * dy_pixels / (MILIMETERS_PER_PIXEL);
    
    OdometerDataUnit metric_data;
    
    metric_data.dx__meters      = MILIMETER_TO_METER(dx_mm);
    metric_data.dy__meters      = MILIMETER_TO_METER(dy_mm);
    metric_data.z_range__meters = MILIMETER_TO_METER(z_height_mm);
    metric_data.dt__sec         = MILISECOND_TO_SECOND(dt_ms);
    metric_data.vx__meters_p_sec         = metric_data.dx__meters / metric_data.dt__sec;
    metric_data.vy__meters_p_sec         = metric_data.dy__meters / metric_data.dt__sec;
    
    return metric_data;
    
}


/*=======================================================
* @brief           Converts odometer data to the lab frame
*
* @param           t  -  The angle between the robot orientation and the lab frame (z-axis)
*
* @param           odom_frame_data   -  Odometer data in the current frame of the device (may be rotated to the lab)
*
* @return          The same odometer data unit rotated by t
*
* @author          Daniel Greenberger
=========================================================*/
inline OdometerDataUnit convert_odom_to_lab_frame (const double t, const OdometerDataUnit odom_frame_data)
{
    OdometerDataUnit lab_frame_data;
    
    /* Rotate distance */
    std::tie( lab_frame_data.dx__meters, lab_frame_data.dy__meters) 
                = rotate_by_angle(t, odom_frame_data.dx__meters, odom_frame_data.dy__meters);
                
    /* Rotate velocity */
    std::tie( lab_frame_data.vx__meters_p_sec, lab_frame_data.vy__meters_p_sec) 
                = rotate_by_angle(t, odom_frame_data.vx__meters_p_sec, odom_frame_data.vy__meters_p_sec);  
    

    /* Time and range are copied */
    lab_frame_data.z_range__meters = odom_frame_data.z_range__meters; // We assume the orientation changes only in the x-y direction
    lab_frame_data.dt__sec         = odom_frame_data.dt__sec;


    
    return lab_frame_data;
}









			
					
}       //  namespace OdometerTransform
#endif  // BCRZ_LINEAR_TO_ROT