//
// Created by zhouhua on 2020/10/15.
//

#include <ActionCircle.hpp>

ActionCircle* ActionCircle::l_pInst = NULL;
ActionCircle::ActionCircle()  :
        m_K_r(PID(K_circle_pid_out, -K_circle_pid_out, 1, 0, 0.05)),
        m_K_h(PID(K_circle_pid_out, -K_circle_pid_out, 1, 0, 0.1)),
        m_K_a(PID(K_circle_pid_out, -K_circle_pid_out, 1, 0, 0.1)){

}

void ActionCircle::onInit(const TCircleConfig &config) {
    m_cfg = config;
    m_output.m_target_heading = config.target_heading;
    m_output.v_out = {0,0,0};
    m_output.m_speed = config.m_speed;
}

void ActionCircle::doProgress(TVec3 &m_curPos) {
// velocity to control radius
    float c_radius = TVec3(m_curPos - m_cfg.m_circle_pos).norm();
    float dt = 0.05; // 20HZ
    if (dt > 0.5) {
        dt = 0.5;
    }
    TVec3 p0 = m_curPos - m_cfg.m_circle_pos;

    float r_speed = K_max_vr * m_K_r.calculate(m_cfg.m_radius, c_radius, dt);
    TVec3 v_r = p0;
    v_r.z() = 0;
    v_r = r_speed * v_r.normalized();

    // velocity to control height
    float c_height = m_curPos.z();
    float h_speed = K_max_vh * m_K_h.calculate(m_cfg.m_target_pos.z(), c_height, dt);
    TVec3 v_h(0, 0, h_speed);

    // velocity to circle
    //get the 2 vector angle
    float rot = m_cfg.is_clockWise ? 90 : -90;
    TQuat quatNED = Calculate::getInstance()->EulerAngle2QuatNED(rot, 0, 0);
    TVec3 dv = m_cfg.m_speed  * (quatNED * p0).normalized();
    dv.z() = 0;
    util_log("dv = (%.2f, %.2f, %.2f)", dv.x(), dv.y(), dv.z());
    m_output.v_out = v_r + v_h + dv;
}

ActionCircle* ActionCircle::getInstance() {
    if (l_pInst == NULL) {
        l_pInst = new ActionCircle();
    }
    return l_pInst;
}

void ActionCircle::GetOutput(TCircleOutput &output) {
    output = m_output;
}
