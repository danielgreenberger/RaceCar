#ifndef BCRZ_LINEAR_TO_ROT
#define BCRZ_LINEAR_TO_ROT

#include <cmath>
#include <utility>

namespace OdometerTransform
{ 
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
                      

                      
                                                              					 
        X = X' * cos(t)  -  Y' * sin(t)
`       Y = X' * sin(t)  +  Y' * cos(t)        
                                                              					 
                                                              					 
                                                              					 
                                                              					 

*/



/*=======================================================
* @brief           Converts distance travelled from   X'-Y'  to  X-Y  frame.
*
* @description     See the diagram above which justifies the calculation. 
*
* @param           t  -  The angle by which X' and Y'  are rotated  
*
* @param           r_Dx   -  The distance in X' units
*
* @param           r_Dy   -  The distance in Y' units
*
* @return          A pair (Dx,Dy)  representing the difference in X and Y  (lab) frame.
*
* @author          Daniel Greenberger
=========================================================*/
using DistXY = std::pair<double,double>;
inline DistXY convert_dist_to_lab_frame (const double t, const double r_Dx, const double r_Dy)
{
    const double dx = r_Dx*cos(t) - r_Dy*sin(t);
    const double dy = r_Dx*sin(t) + r_Dy*cos(t);
    
    return std::make_pair(dx,dy);
}

 

			
					
}       //  namespace OdometerTransform
#endif  // BCRZ_LINEAR_TO_ROT