#include "pid_control.h"

PIDController::PIDController(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->previousError = 0;
    this->integral = 0;
}

float PIDController::calculate(float error) {
    integral += error;
    float derivative = error - previousError;
    float output = kp * error + ki * integral + kd * derivative;
    previousError = error;
    return output;
}
