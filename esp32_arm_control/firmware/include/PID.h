#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp, float ki, float kd, float max_out);
    float compute(float setpoint, float input, float dt);
    void setGains(float kp, float ki, float kd);
    void getGains(float &kp, float &ki, float &kd) const;
    void setMaxIntegral(float max_integral);
    void reset();

private:
    float _kp, _ki, _kd;
    float _max_out;
    float _max_integral;
    float _integral;
    float _prev_error;
    float _prev_input; // For derivative on measurement
    float _filtered_d; // Low-pass filtered derivative
};

#endif
