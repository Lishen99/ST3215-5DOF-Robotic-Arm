#include "PID.h"
#include <math.h>

PID::PID(float kp, float ki, float kd, float max_out) 
    : _kp(kp), _ki(ki), _kd(kd), _max_out(max_out), 
      _max_integral(max_out / 2.0f),
      _integral(0), _prev_error(0), _prev_input(0), _filtered_d(0) {}

void PID::setGains(float kp, float ki, float kd) {
    _kp = kp; 
    _ki = ki; 
    _kd = kd;
    _integral = 0;
}

void PID::getGains(float &kp, float &ki, float &kd) const {
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

void PID::setMaxIntegral(float max_integral) {
    _max_integral = max_integral;
}

void PID::reset() {
    _integral = 0;
    _prev_error = 0;
    _prev_input = 0;
    _filtered_d = 0;
}

float PID::compute(float setpoint, float input, float dt) {
    if (dt < 0.0001f) dt = 0.0001f;
    if (dt > 0.1f) dt = 0.1f;
    
    float error = setpoint - input;
    
    // Proportional
    float p_out = _kp * error;
    
    // Integral with anti-windup
    _integral += error * dt;
    if (_integral > _max_integral) _integral = _max_integral;
    if (_integral < -_max_integral) _integral = -_max_integral;
    float i_out = _ki * _integral;
    
    // Derivative on measurement with low-pass filter
    float d_input = (input - _prev_input) / dt;
    float d_alpha = 0.3f;
    _filtered_d = d_alpha * d_input + (1.0f - d_alpha) * _filtered_d;
    float d_out = -_kd * _filtered_d;
    
    float output = p_out + i_out + d_out;
    
    // Clamp output
    if (output > _max_out) output = _max_out;
    if (output < -_max_out) output = -_max_out;
    
    // Back-calculation anti-windup
    if ((output >= _max_out && error > 0) || (output <= -_max_out && error < 0)) {
        _integral -= error * dt * 0.5f;
    }
    
    _prev_error = error;
    _prev_input = input;
    
    return output;
}
