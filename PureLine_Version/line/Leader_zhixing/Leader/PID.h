
#ifndef _PID_H
#define _PID_H

class PID_c {
  public:

    float last_error;
    float prev_error;
    float p_term;
    float i_term;
    float d_term;
    float feedback;

    float p_gain;
    float i_gain;
    float d_gain;

    float error_deadzone;
    float output_deadzone;

    float output_min;
    float output_max;
    float max_delta;
    float output_filter;
    
    float min_effective_output;
    float zero_threshold;

    unsigned long ms_last_t;
  
    PID_c() {
    } 

    void initialise( float p, float i, float d ) {
       feedback = 0;
       last_error = 0;
       prev_error = 0;
       p_term = 0;
       i_term = 0;
       d_term = 0;

       p_gain = p;
       i_gain = i;
       d_gain = d;

       error_deadzone = 0.0f;
       output_deadzone = 0.0f;
       
       output_min = -255.0f;
       output_max = 255.0f;
       max_delta = 999.0f;
       output_filter = 1.0f;
       
       min_effective_output = 0.0f;
       zero_threshold = 0.0f;

       ms_last_t = millis();
    }

    void setDeadzone( float error_dz, float output_dz ) {
       error_deadzone = error_dz;
       output_deadzone = output_dz;
    }

    void setOutputLimits( float min_output, float max_output ) {
       output_min = min_output;
       output_max = max_output;
    }

    void setMaxDelta( float max_change ) {
       max_delta = max_change;
    }

    void setOutputFilter( float alpha ) {
       if( alpha < 0.0f ) alpha = 0.0f;
       if( alpha > 1.0f ) alpha = 1.0f;
       output_filter = alpha;
    }

    void setMinEffectiveOutput( float zero_th, float min_eff ) {
       zero_threshold = zero_th;
       min_effective_output = min_eff;
    }

    void reset() {
      p_term = 0;
      i_term = 0;
      d_term = 0;
      last_error = 0;
      prev_error = 0;
      feedback = 0;
      ms_last_t = millis();
    }

    float update( float demand, float measurement ) {
      float error;
      unsigned long ms_now_t;
      unsigned long ms_dt;
      float float_dt;
      float delta_output;

      ms_now_t = millis();
      ms_dt = ms_now_t - ms_last_t;

      ms_last_t = millis();
      
      float_dt = (float)ms_dt;

      if( float_dt == 0.0 ) return feedback;

      error = demand - measurement;

      if( error_deadzone > 0.0f && fabs(error) < error_deadzone ) {
        error = 0.0f;
      }

      p_term = p_gain * (error - last_error);

      i_term = i_gain * error * float_dt;
      
      d_term = d_gain * (error - 2.0f * last_error + prev_error) / float_dt;

      delta_output = p_term + i_term + d_term;

      if( output_deadzone > 0.0f && fabs(delta_output) < output_deadzone ) {
        delta_output = 0.0f;
      }

      if( max_delta < 999.0f ) {
        if( delta_output > max_delta ) {
          delta_output = max_delta;
        } else if( delta_output < -max_delta ) {
          delta_output = -max_delta;
        }
      }

      float old_feedback = feedback;

      feedback += delta_output;

      if( feedback > output_max ) {
        feedback = output_max;
      } else if( feedback < output_min ) {
        feedback = output_min;
      }

      if( output_filter < 1.0f ) {
        feedback = output_filter * feedback + (1.0f - output_filter) * old_feedback;
      }

      if( min_effective_output > 0.0f ) {
        float abs_fb = fabs(feedback);
        
        if( abs_fb < zero_threshold ) {
          feedback = 0.0f;
        }
        else if( abs_fb < min_effective_output ) {
          if( feedback > 0 ) {
            feedback = min_effective_output;
          } else {
            feedback = -min_effective_output;
          }
        }
      }

      prev_error = last_error;
      last_error = error;

      return feedback;
    }

};

#endif
