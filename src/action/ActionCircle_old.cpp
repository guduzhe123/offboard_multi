/*
//
// Created by zhouhua on 2020/10/15.
//

#include <ActionCircle.hpp>

ActionCircle* ActionCircle::l_pInst = NULL;
ActionCircle::ActionCircle()  :
        m_K_r(PID(K_circle_pid_out, -K_circle_pid_out, 1, 0, 0.05)),
        m_K_h(PID(K_circle_pid_out, -K_circle_pid_out, 1, 0, 0.1)),
        m_K_a(PID(K_circle_pid_out, -K_circle_pid_out, 0.05, 0.0, 0.005)){

}

void ActionCircle::onInit(const TCircleConfig &config) {
    m_cfg = config;
    m_output.m_target_heading = config.target_heading;
    m_output.v_out = {0,0,0};
    m_output.m_speed = config.m_speed;
}

void ActionCircle::doProgress(TVec3 &m_curPos, float cur_heading) {
// velocity to control radius
    float c_radius = TVec3(m_curPos - m_cfg.m_circle_pos).norm();
    float dt = 0.05; // 20HZ
    if (dt > 0.5) {
        dt = 0.5;
    }
    TVec3 p0 = m_curPos - m_cfg.m_circle_pos;
    chlog::info("data",circle m_curPos = (%.2f, %.2f, %.2f), m_circle_pos = (%.2f, %.2f, %.2f)", m_curPos.x(), m_curPos.y(), m_curPos.z()
            , m_cfg.m_circle_pos.x(), m_cfg.m_circle_pos.y(), m_cfg.m_circle_pos.z());

    float r_speed = K_max_vr * m_K_r.calculate(m_cfg.m_radius, c_radius, dt);
    TVec3 v_r = p0;
    v_r.z() = 0;
    v_r = r_speed * v_r.normalized();

    // velocity to control height
    float c_height = m_curPos.z();
    float h_speed = K_max_vh * m_K_h.calculate(m_cfg.m_target_pos.z(), c_height, dt);
//    TVec3 v_h(0, 0, h_speed);
    TVec3 v_h(0, 0, 0);

    // velocity to circle
    //get the 2 vector angle
    float rot = m_cfg.is_clockWise ? 90 : -90;
    TQuat quatNED = Calculate::getInstance()->EulerAngle2QuatNED(rot, 0, 0);
    TVec3 dv = m_cfg.m_speed  * (quatNED * p0).normalized();
    dv.z() = 0;
    chlog::info("data",dv = (%.2f, %.2f, %.2f)", dv.x(), dv.y(), dv.z());
    m_output.v_out = v_r + v_h + dv;

    TVec3 nv = p0.normalized();
    chlog::info("data",nv = (%.2f, %.2f, %.2f)", nv.x(), nv.y(), nv.z());
    float heading = rad2dgr(atan2(-nv.y(), nv.x()));
//    m_output.m_target_heading = dgrIn180s(heading);
    float rate = K_max_va * m_K_a.calculate(90, cur_heading, dt);
    m_output.m_yaw_rate = rate;
    chlog::info("data",m_output.m_target_heading = %.2f, cur_heading = %.2f, rate = %.2f",
            m_output.m_target_heading, cur_heading, rate);

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

float ActionCircle::dgrIn180s(float d) {
    int n = d / 360;
    float d1 = d - (float) (n * 360);
    if (d1 > 180) d1 -= 360;
    if (d1 < -180) d1 += 360;
    return d1;
}
*/
