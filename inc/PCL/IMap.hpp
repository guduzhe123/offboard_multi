//
// Created by zhouhua on 2020/12/22.
//

#ifndef WINDAPPCORE_IMAP_HPP
#define WINDAPPCORE_IMAP_HPP

#include "Cinc.hpp"

class IMap {
public:
    IMap() = default;
    virtual ~IMap() {};

    virtual void onInit() = 0;

    virtual bool isStateValid(const Eigen::Vector3f &PosENU) = 0;

    virtual void updateOctomap(const octomap_msgs::Octomap &msg) = 0;

    virtual void setSafeRaduis(const float &raduis) = 0;
};
#endif //WINDAPPCORE_IMAP_HPP
