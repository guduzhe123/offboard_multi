//
// Created by zhouhua on 2020/1/3.
//

#ifndef OFFBOARD_IMSGROSMANAGER_HPP
#define OFFBOARD_IMSGROSMANAGER_HPP

#include "Cinc.hpp"

class IMsgRosManager {
public:
    virtual  ~IMsgRosManager() {};

    virtual void OnInit() = 0;

};
#endif //OFFBOARD_IMSGROSMANAGER_HPP
