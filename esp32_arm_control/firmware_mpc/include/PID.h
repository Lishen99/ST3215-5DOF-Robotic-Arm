#ifndef PID_H
#define PID_H

/**
 * PID Controller with anti-windup and derivative filtering
 * Used as low-level velocity controller for each servo
 */
class PID {
public:
    PID(float kp = 10.0f, float ki = 0.0f, float kd = 0.5f, float max_out = 1500.0f);
    
    /**
     * Compute PID output
     * @param setpoint Target value
     * @param input Current value
     * @param dt Time step (seconds)
     * @return Control output (velocity command)
     */
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
    float _prev_input;
    float _filtered_d;
};

#endif
