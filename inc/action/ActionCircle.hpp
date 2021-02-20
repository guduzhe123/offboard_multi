//
// Created by zhouhua on 2020/10/15.
//

#ifndef OFFBOARD_ACTIONCIRCLE_HPP
#define OFFBOARD_ACTIONCIRCLE_HPP

#include "Cinc.hpp"
#include "pid.hpp"
#include "Calculate.hpp"

static const float K_circle_pid_out = 1.0f;
static const float K_max_vh = 0.5f; //max speed to control height
static const float K_max_vr = 0.5f; //max speed to control radius
static const float K_max_va = 10.0f; //max speed to control angle
static const float K_circle_distance_range = 0.8f;
static const float K_heading_err = 2.0f;

class ActionCircle {
public:
    ActionCircle();
    ~ActionCircle() = default;

    void onInit(const TCircleConfig &config);
    void doProgress(TVec3 &m_curPos, float cur_heading);
    void GetOutput(TCircleOutput& output);
    static ActionCircle* getInstance();

private:
    float rad2dgr(float d) { return d * 180.0 / M_PI; };
    float dgrIn180s(float d);
    void getCircleTarget(float angle);

    TCircleOutput m_output;
    TCircleConfig m_cfg;
    TCircleStatus m_status;
    // PID to control radius
    PID m_K_r;
    // PID to control height
    PID m_K_h;
    // PID to control angle
    PID m_K_a;

    static ActionCircle* l_pInst;
};
#endif //OFFBOARD_ACTIONCIRCLE_HPP
