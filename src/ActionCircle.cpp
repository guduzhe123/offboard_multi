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

    getCircleTarget(M_PI *1 / 4);
    getCircleTarget(M_PI * 1 / 2);
    getCircleTarget(M_PI *3 / 4);
    getCircleTarget(M_PI );
    getCircleTarget(M_PI * 5 / 4);
    getCircleTarget(M_PI * 6 / 4);
    getCircleTarget(M_PI * 7 / 4);
    getCircleTarget(M_PI * 8 / 4);
    getCircleTarget(M_PI * 9 / 4);
    getCircleTarget(M_PI * 10 / 4);
    getCircleTarget(M_PI * 11 / 4);
    getCircleTarget(M_PI * 12 / 4);
    getCircleTarget(M_PI * 13 / 4);
    getCircleTarget(M_PI * 14 / 4);
    getCircleTarget(M_PI * 15 / 4);
    getCircleTarget(M_PI * 16 / 4);

}

void ActionCircle::getCircleTarget(float angle) {
    TVec3 center_start_vec = m_cfg.m_circle_pos - m_cfg.m_start_pos;
    Eigen::AngleAxisf rotation_vector(angle, Eigen::Vector3f(0, 0, 1));
    cout << "Rotation_vector1" << endl << rotation_vector.matrix() << endl;
    TVec3 target = m_cfg.m_circle_pos - rotation_vector.matrix().inverse() * center_start_vec ;
    m_output.circle_target.push_back(target);
    geometry_msgs::PoseStamped wp;
    wp.pose.position.x = target.x();
    wp.pose.position.y = target.y();
    wp.pose.position.z = target.z();
    m_output.usv_way_points_.push_back(wp);
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
