//
// Created by zhouhua on 2020/12/22.
//

#ifndef OFFBOARD_IMAP_HPP
#define OFFBOARD_IMAP_HPP

#include <Eigen/Core>
#include <octomap_msgs/Octomap.h>

class IMap {
public:
    IMap() = default;
    virtual ~IMap() {};

    virtual void onInit() = 0;

    virtual bool isStateValid(const Eigen::Vector3f &PosENU, bool planning) = 0;

    virtual void updateOctomap(const octomap_msgs::Octomap &msg) = 0;

    virtual void setSafeRaduis(const float &raduis) = 0;

    virtual void getMinDistance(const Eigen::Vector3f& cur_pos, float& min_dist) = 0;
};
#endif //OFFBOARD_IMAP_HPP
