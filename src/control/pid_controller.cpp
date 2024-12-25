#include "control/pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd) 
    : Kp(kp), Ki(ki), Kd(kd), previousError(0), integral(0) {}

float PIDController::computeHeaterOutput(float currentTemp, float targetTemp) {
    float error = targetTemp - currentTemp;
    integral += error;
    float derivative = error - previousError;
    previousError = error;
    
    return Kp * error + Ki * integral + Kd * derivative;
}

float PIDController::computeCoolerOutput(float currentTemp, float targetTemp) {
    float error = currentTemp - targetTemp;
    integral += error;
    float derivative = error - previousError;
    previousError = error;
    
    return Kp * error + Ki * integral + Kd * derivative;
}
