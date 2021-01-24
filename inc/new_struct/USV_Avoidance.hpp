//
// Created by zhouhua on 19-12-13.
//

#ifndef OFFBOARD_USV_AVOIDANCE_HPP
#define OFFBOARD_USV_AVOIDANCE_HPP

#include "Cinc.hpp"
#include "DataMan.hpp"
#include "IControlFunction.hpp"
#include "Calculate.hpp"

static float K_usv_avodiance_dist = 4;
class USV_Avoidance : public IControlFunction {
public:
    USV_Avoidance();

    ~USV_Avoidance() {};

    void Oninit(const int config) override;

    void GetData() override ;

    void DoProgress() override ;

    void SetFunctionOutPut() override;

    void checkHorizontalDistance(const multi_vehicle_vec &vehicles);

    void Getvehicledistance(const M_Drone &vehicle1, const M_Drone &vehicle2, float &distance);

    static  USV_Avoidance* getInstance();

private:
    void checkDistance(const M_Drone &vehicle1, const M_Drone &vehicle2);


    static USV_Avoidance* l_lptr;

    bool is_run_avoidance_;

    multi_vehicle m_multi_vehicle_{};
    int config_;

    bool is_usv1_usv2_crash_;
    bool is_usv1_usv3_crash_;
    bool is_usv2_usv3_crash_;

    bool usv1_crash_;
    bool usv2_crash_;
    bool usv3_crash_;

};

class USVAvoidanceFactory : public IFunctionFactory {
public:
    ~USVAvoidanceFactory() {};

    IControlFunction* FunctionCreator()  {
        return USV_Avoidance::getInstance();
    } ;

};

#endif //OFFBOARD_USV_AVOIDANCE_HPP
