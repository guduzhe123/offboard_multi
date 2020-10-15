#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.hpp"

using namespace std;

/**
 * Implementation
 */

PID::PID(float max, float min, float Kp, float Kd, float Ki) :
        _max(max),
        _min(min),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki),
        _pre_error(0),
        _integral(0) {
}

float PID::calculate(float setpoint, float pv, float dt) {

    // Calculate error
    float error = setpoint - pv;

    // Proportional term
    float Pout = _Kp * error;

    // Integral term
    _integral += error * dt;
    float Iout = _Ki * _integral;
    // Restrict to max/min
    if (Iout > _max)
        Iout = _max;
    else if (Iout < _min)
        Iout = _min;

    // Derivative term
    float derivative = (error - _pre_error) / dt;
    float Dout = _Kd * derivative;

    // Calculate total output
    float output = Pout + Iout + Dout;

    // Restrict to max/min
    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

void PID::pid_config(float new_Imax, float new_P, float new_I, float new_D) {
    _max = new_Imax;
    _min = -new_Imax;
    _Kp = new_P;
    _Ki = new_I;
    _Kd = new_D;
}

PID::~PID() {
}

void PID::reset_integral() {
    _integral = 0;
}

#endif
