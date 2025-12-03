
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <math.h>

extern volatile long count_e0;
extern volatile long count_e1;

const float count_per_rev = 358.3;
const float wheel_radius  = 17.475;
const float wheel_sep     = 44.48;

const float mm_per_count  = ( 2.0 * wheel_radius * PI ) / count_per_rev;

class Kinematics_c {
  public:

    float x,y,theta;

    long last_e1;
    long last_e0;
  
    Kinematics_c() {

    } 

    void initialise( float start_x, float start_y, float start_th ) {
      last_e0 = count_e0;
      last_e1 = count_e1;
      x = start_x;
      y = start_y;
      theta = start_th;
    }
    
    void update( ) {
      
        long delta_e1;
        long delta_e0;
        float mean_delta;
         
        float x_contribution;
        float th_contribution;

        delta_e1 = count_e1 - last_e1;
        delta_e0 = count_e0 - last_e0;

        last_e1 = count_e1;
        last_e0 = count_e0;
        
        mean_delta = (float)delta_e1;
        mean_delta += (float)delta_e0;
        mean_delta /= 2.0;

        x_contribution = mean_delta * mm_per_count;

        th_contribution = (float)delta_e0;
        th_contribution -= (float)delta_e1;
        th_contribution *= mm_per_count;
        th_contribution /= (wheel_sep *2.0);

        x = x + x_contribution * cos( theta );
        y = y + x_contribution * sin( theta );
        theta = theta + th_contribution;

    }

};

#endif
