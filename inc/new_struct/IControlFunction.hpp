//
// Created by zhouhua on 2020/1/28.
//

#ifndef OFFBOARD_ICONTROLFUNCTION_HPP
#define OFFBOARD_ICONTROLFUNCTION_HPP

#include "Cinc.hpp"

class IControlFunction {
public:
    virtual ~IControlFunction() {};

    virtual void Oninit(const int config) = 0;

    virtual void GetData() = 0;

    virtual void DoProgress() = 0;

    virtual void SetFunctionOutPut() = 0;
};


class IFunctionFactory {
public:
    virtual ~IFunctionFactory() {};

    virtual IControlFunction* FunctionCreator() = 0;
};

#endif //OFFBOARD_ICONTROLFUNCTION_HPP
