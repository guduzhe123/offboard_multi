//
// Created by zhouhua on 19-12-1.
//

#include "Multi_formation.hpp"

MultiFormation* MultiFormation::multi_formation = NULL;

MultiFormation::MultiFormation() {

}

void MultiFormation::Oninit(const float config) {

}

void MultiFormation::setEachLoclation() {

}

MultiFormation* MultiFormation::getInstance() {
    if (multi_formation == NULL) {
        multi_formation = new MultiFormation();
    }
    return multi_formation;
}