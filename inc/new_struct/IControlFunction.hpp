//
// Created by zhouhua on 2020/1/28.
//

#ifndef OFFBOARD_ICONTROLFUNCTION_HPP
#define OFFBOARD_ICONTROLFUNCTION_HPP

#include "Cinc.hpp"

class IControlFunction {
public:
    virtual ~IControlFunction() = 0;

    virtual void GetData() = 0;

    virtual void DoProgress() = 0;

    virtual void SetFunctionOutPut() = 0;
};

#endif //OFFBOARD_ICONTROLFUNCTION_HPP
