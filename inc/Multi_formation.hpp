//
// Created by zhouhua on 19-12-1.
//

#ifndef OFFBOARD_MULTI_FORMATION_HPP
#define OFFBOARD_MULTI_FORMATION_HPP

#include "Cinc.hpp"



class MultiFormation {
public:
    MultiFormation();

    ~MultiFormation() = default;

    void Oninit(const float config);

    static MultiFormation* getInstance();

private:

    static MultiFormation* multi_formation;

    void setEachLoclation();

};
#endif //OFFBOARD_MULTI_FORMATION_HPP
