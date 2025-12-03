
#ifndef _PID_H
#define _PID_H

// 增量式PID控制器
class PID_c {
  public:

    float last_error;      // 上一次误差 e(k-1)
    float prev_error;      // 上上次误差 e(k-2)
    float p_term;          // 比例项
    float i_term;          // 积分项
    float d_term;          // 微分项
    float feedback;        // 当前总输出值（累积）

    float p_gain;          // 比例增益 Kp
    float i_gain;          // 积分增益 Ki
    float d_gain;          // 微分增益 Kd

    float error_deadzone;  // 误差死区阈值
    float output_deadzone; // 输出死区阈值

    float output_min;      // 输出最小值限制
    float output_max;      // 输出最大值限制
    float max_delta;       // 最大输出变化率（每次最大变化量）
    float output_filter;   // 输出低通滤波系数 (0-1)，越小越平滑
    
    float min_effective_output;  // 最小有效输出（电机启动阈值）
    float zero_threshold;        // 接近0的阈值

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

       error_deadzone = 0.0f;   // 默认无误差死区
       output_deadzone = 0.0f;  // 默认无输出死区
       
       output_min = -255.0f;    // 默认PWM最小值
       output_max = 255.0f;     // 默认PWM最大值
       max_delta = 999.0f;      // 默认无变化率限制
       output_filter = 1.0f;    // 默认无滤波（1.0=不滤波）
       
       min_effective_output = 0.0f;  // 默认无最小有效输出限制
       zero_threshold = 0.0f;        // 默认无零点阈值

       ms_last_t = millis();
    }

    // 设置死区参数
    void setDeadzone( float error_dz, float output_dz ) {
       error_deadzone = error_dz;
       output_deadzone = output_dz;
    }

    // 设置输出限幅
    void setOutputLimits( float min_output, float max_output ) {
       output_min = min_output;
       output_max = max_output;
    }

    // 设置输出变化率限制（防止突变）
    void setMaxDelta( float max_change ) {
       max_delta = max_change;
    }

    // 设置输出滤波系数（0.0-1.0，越小越平滑，建议0.3-0.7）
    void setOutputFilter( float alpha ) {
       if( alpha < 0.0f ) alpha = 0.0f;
       if( alpha > 1.0f ) alpha = 1.0f;
       output_filter = alpha;
    }

    // 设置最小有效输出（电机启动阈值，例如10）
    // zero_th: 接近0的阈值（例如5），小于此值强制为0
    // min_eff: 最小有效输出（例如10），大于zero_th但小于min_eff时强制为min_eff
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
      // 注意：死区参数不重置，保持用户设定的值
    }

    // 增量式PID算法
    // 输出增量 Δu(k) = Kp[e(k)-e(k-1)] + Ki*e(k) + Kd[e(k)-2e(k-1)+e(k-2)]
    float update( float demand, float measurement ) {
      float error;
      unsigned long ms_now_t;
      unsigned long ms_dt;
      float float_dt;
      float delta_output;  // 输出增量

      ms_now_t = millis();
      ms_dt = ms_now_t - ms_last_t;

      ms_last_t = millis();
      
      float_dt = (float)ms_dt;

      if( float_dt == 0.0 ) return feedback;

      // 当前误差
      error = demand - measurement;

      // 误差死区处理：如果误差在死区内，视为0
      if( error_deadzone > 0.0f && fabs(error) < error_deadzone ) {
        error = 0.0f;
      }

      // 增量式PID三项计算
      // P项增量：Kp * [e(k) - e(k-1)]
      p_term = p_gain * (error - last_error);

      // I项增量：Ki * e(k) * dt
      i_term = i_gain * error * float_dt;
      
      // D项增量：Kd * [e(k) - 2*e(k-1) + e(k-2)] / dt
      d_term = d_gain * (error - 2.0f * last_error + prev_error) / float_dt;

      // 计算输出增量
      delta_output = p_term + i_term + d_term;

      // 输出死区处理：如果增量太小，不进行调整
      if( output_deadzone > 0.0f && fabs(delta_output) < output_deadzone ) {
        delta_output = 0.0f;
      }

      // 输出变化率限制：防止输出突变（平滑加速）
      if( max_delta < 999.0f ) {
        if( delta_output > max_delta ) {
          delta_output = max_delta;
        } else if( delta_output < -max_delta ) {
          delta_output = -max_delta;
        }
      }

      // 保存旧输出用于滤波
      float old_feedback = feedback;

      // 累加到总输出（增量式特点：输出 = 上次输出 + 本次增量）
      feedback += delta_output;

      // 输出限幅：确保在合理范围内
      if( feedback > output_max ) {
        feedback = output_max;
      } else if( feedback < output_min ) {
        feedback = output_min;
      }

      // 低通滤波：平滑输出（一阶滤波）
      // feedback = alpha * new_value + (1-alpha) * old_value
      if( output_filter < 1.0f ) {
        feedback = output_filter * feedback + (1.0f - output_filter) * old_feedback;
      }

      // 最小有效输出处理（防止电机抖动）
      // 这是你同学的方法：接近0就=0，接近10就=10
      if( min_effective_output > 0.0f ) {
        float abs_fb = fabs(feedback);
        
        // 如果输出很小（小于零点阈值），强制为0
        if( abs_fb < zero_threshold ) {
          feedback = 0.0f;
        }
        // 如果输出在零点阈值和最小有效输出之间，强制提升到最小有效输出
        else if( abs_fb < min_effective_output ) {
          // 保持原来的正负号
          if( feedback > 0 ) {
            feedback = min_effective_output;
          } else {
            feedback = -min_effective_output;
          }
        }
      }

      // 更新误差历史
      prev_error = last_error;
      last_error = error;

      return feedback;
    }

};

#endif
