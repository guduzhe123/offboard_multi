//
// Created by zhouhua on 2021/2/9.
//

#include "PCL/OctoMap.hpp"
OctoMap::OctoMap() :
        safety_radius_(6) {

}

void OctoMap::onInit() {

    //四旋翼的障碍物几何形状
    Quadcopter_ = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Sphere(safety_radius_));
    //分辨率参数设置
    fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.5)));
    tree_obj_ = std::shared_ptr<fcl::CollisionGeometry>(tree);
}

void OctoMap::updateOctomap(const octomap_msgs::Octomap &msg) {
    if (!msg.data.empty()) {
        octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(msg));
        fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));

        // Update the octree used for collision checking
        tree_obj_ = std::shared_ptr<fcl::CollisionGeometry>(tree);
    }
}

void OctoMap::setSafeRaduis(const float &raduis) {
    safety_radius_ = raduis;
}

bool OctoMap::isStateValid(const Eigen::Vector3f &PosENU, float &min_dist)
{
    // cast the abstract state type to the type we expect
    Quadcopter_ = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Sphere(safety_radius_));
    fcl::CollisionObject treeObj((tree_obj_));
    fcl::CollisionObject aircraftObject(Quadcopter_);

    // check validity of state defined by pos & rot
    fcl::Vec3f translation(PosENU.x(), PosENU.y(), PosENU.z());
    aircraftObject.setTransform(translation);
    fcl::CollisionRequest requestType(1,false,1,false);
    fcl::CollisionResult collisionResult;
    fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

    min_dist = 5.5;

    return(!collisionResult.isCollision());
}
