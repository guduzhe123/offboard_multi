#ifndef _PID_HPP_
#define _PID_HPP_
//------------------------------
//  PID
//------------------------------
/*
 Modified from :
    https://gist.github.com/bradley219/5373998
 */
class PID {
public:

    //-------- cfg
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // dt -  loop interval time
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable
    struct TCfg {
        TCfg() {};

        TCfg(float max, float min, float Kp, float Kd, float Ki) :
                max(max), min(min), Kp(Kp), Ki(Ki), Kd(Kd) {}

        float max = 0;
        float min = 0;
        float Kp = 0;
        float Ki = 0;
        float Kd = 0;
    };

    PID(float max, float min, float Kp, float Kd, float Ki);

    void setCfg(float max, float min,
                float Kp, float Kd, float Ki);

    void setCfg(const TCfg &cfg) {
        setCfg(cfg.max, cfg.min, cfg.Kp, cfg.Kd, cfg.Ki);
    }

    // Returns the manipulated variable given a setpoint and current process value
    float calculate(float setpoint, float pv, float dt);

    void pid_config(float new_Imax, float new_P, float new_I, float new_D);

    void reset_integral();

    ~PID();

private:
    float _max;
    float _min;
    float _Kp;
    float _Kd;
    float _Ki;
    float _pre_error;
    float _integral;
};

#endif
