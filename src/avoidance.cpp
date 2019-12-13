//
// Created by zhouhua on 19-12-1.
//

#include "avoidance.hpp"

Avoidance* Avoidance::l_lptr = NULL;

Avoidance::Avoidance() {

}

void Avoidance::Oninit() {

}

Avoidance* Avoidance::getInstance() {
    if (l_lptr == NULL) {
        l_lptr = new Avoidance();
    }
    return l_lptr;
}

void Avoidance::usv_avoidance() {

}

void Avoidance::uav_avoidance(const M_Drone &m_drone, M_Drone_Avoidace &m_drone_avoidance) {


}
