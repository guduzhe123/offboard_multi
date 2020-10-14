#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;
namespace ext {


    class PIDImpl {
    public:
        PIDImpl(float max, float min, float Kp, float Kd, float Ki);

        ~PIDImpl();

        float calculate(float setpoint, float pv, float dt);

        void reset_integral();

        void pid_config(float new_Imax, float new_P, float new_I, float new_D);

        void setCfg(float max, float min,
                    float Kp, float Kd, float Ki) {
            _max = max;
            _min = min;
            _Kp = Kp;
            _Ki = Ki;
            _Kd = Kd;
        }

    private:
        float _max;
        float _min;
        float _Kp;
        float _Kd;
        float _Ki;
        float _pre_error;
        float _integral;
    };


    PID::PID(float max, float min, float Kp, float Kd, float Ki) {
        pimpl = new PIDImpl(max, min, Kp, Kd, Ki);
    }

    void PID::setCfg(float max, float min,
                     float Kp, float Kd, float Ki) {
        if (pimpl != nullptr)
            pimpl->setCfg(max, min, Kp, Kd, Ki);
    }

    float PID::calculate(float setpoint, float pv, float dt) {
        return pimpl->calculate(setpoint, pv, dt);
    }

    void PID::pid_config(float new_Imax, float new_P, float new_I, float new_D) {
        pimpl->pid_config(new_Imax, new_P, new_I, new_D);
    }

    void PID::reset_integral(){
        pimpl->reset_integral();
    }

    PID::~PID() {
        delete pimpl;
    }


    /**
     * Implementation
     */
    PIDImpl::PIDImpl(float max, float min, float Kp, float Kd, float Ki) :
            _max(max),
            _min(min),
            _Kp(Kp),
            _Kd(Kd),
            _Ki(Ki),
            _pre_error(0),
            _integral(0) {
    }

    float PIDImpl::calculate(float setpoint, float pv, float dt) {

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

    void PIDImpl::pid_config(float new_Imax, float new_P, float new_I, float new_D) {
        _max = new_Imax;
        _min = -new_Imax;
        _Kp = new_P;
        _Ki = new_I;
        _Kd = new_D;
    }

    PIDImpl::~PIDImpl() {
    }

    void PIDImpl::reset_integral() {
        _integral = 0;
    }
}

#endif