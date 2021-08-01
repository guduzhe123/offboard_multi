//
// Created by zhouhua on 2021/2/9.
//

#ifndef OFFBOARD_OCTOMAP_HPP
#define OFFBOARD_OCTOMAP_HPP

#include "Cinc.hpp"
#include "visualization_msgs/Marker.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/distance.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"
#include "fcl/BV/AABB.h"

#include "DataMan.hpp"
#include "IMap.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;
class OctoMap  : public IMap {
public:
    OctoMap();
    ~OctoMap() = default;

    void onInit() override;
    void updateOctomap(const octomap_msgs::Octomap &msg) override;
    bool isStateValid(const Eigen::Vector3f &PosENU, bool planning) override ;
    void setSafeRaduis(const float &raduis) override;
    void getMinDistance(const Eigen::Vector3f& cur_pos, float& min_dist) override ;
    typedef unique_ptr<OctoMap> Ptr;
private:

    float safety_radius_;
    std::shared_ptr<fcl::CollisionGeometry> Quadcopter_;
    std::shared_ptr<fcl::CollisionGeometry> tree_obj_;
    std::shared_ptr<fcl::CollisionObject> aircraftObject;
};
#endif //OFFBOARD_OCTOMAP_HPP
