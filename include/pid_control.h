#ifndef PID_CONTROL_H
#define PID_CONTROL_H

class PIDController {
private:
    float kp, ki, kd;
    float previousError;
    float integral;

public:
    PIDController(float kp, float ki, float kd);
    float calculate(float error);
};

#endif
