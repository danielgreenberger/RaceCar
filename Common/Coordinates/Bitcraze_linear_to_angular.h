#ifndef BCRZ_LINEAR_TO_ROT
#define BCRZ_LINEAR_TO_ROT

#include <cmath>
        
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
					 
			
			t is the angle of rotation.
			
		    (Dx)/(Dr) = cos(t)
		    (Dy)/(Dr) = sin(t)	
			
		
		    For Dy = 0 we get  t = 0  since  Dr = Dx 
		   
     	    For Dy != 0:
				(Dx)/(Dy) = cos(t) / sin(t)  = cot(t)	
			=> 	t = arccot (Dx/Dy)
			
			Since for every angle w ->  arccot(w) = PI/2 - arctan(w):
			=>  t = PI/2 - arctan(Dx/Dy)
			
*/

inline double get_angle_from_linear_offset(const double Dx, const double Dy)
{
    int angular_movement_radians;
    
    if (Dy == 0)
    {
        angular_movement_radians = 0;
    }
    else
    {
        angular_movement_radians = M_PI/2 - atan(Dx/Dy);
    }
}

			
					

#endif  // BCRZ_LINEAR_TO_ROT