//
// Created by zhouhua on 19-12-13.
//

#ifndef OFFBOARD_AVOIDANCE_HPP
#define OFFBOARD_AVOIDANCE_HPP

#include "Cinc.hpp"
#include "DataMan.hpp"
#include "IControlFunction.hpp"
#include "Calculate.hpp"

static float K_max_avodiance_pos_ = 20;
static float K_avodiance_safe_pos_ = 2;
class Avoidance : public IControlFunction {
public:
    Avoidance();

    ~Avoidance() {};

    void Oninit();

    void GetData() override ;

    void DoProgress() override ;

    void SetFunctionOutPut() override;

    void uav_avoidance();

    void usv_avoidance();

//    void get_uav_avo_output(vector<M_Drone_Avoidace> &m_drone_avoidance);

    void checkVerticalDistance(const multi_vehicle_vec &vehicles);

    void checkHorizontalDistance(const multi_vehicle_vec &vehicles);

    bool checkHorizontalArrive(const M_Drone &vehicle);

    void Getvehicledistance(const M_Drone &vehicle1, const M_Drone &vehicle2, float &distance_h, float &distance_v);

    static  Avoidance* getInstance();

private:
    void checkDistance(const M_Drone &vehicle1, const M_Drone &vehicle2,
                       float &m_drone_avoidance1, float &m_drone_avoidance2);


    static Avoidance* l_lptr;

    bool is_run_avoidance_;
    float distance_h_12_ = 0;
    float distance_h_13_ = 0;
    float distance_h_14_ = 0;
    float distance_h_23_ = 0;
    float distance_h_24_ = 0;
    float distance_h_34_ = 0;

    multi_vehicle_vec multi_vehicle_{};

    M_Drone_Avoidace height_avoidance_uav1_;
    M_Drone_Avoidace height_avoidance_uav2_;
    M_Drone_Avoidace height_avoidance_uav3_;
    M_Drone_Avoidace height_avoidance_uav4_;

};

class AvoidanceFactory : public IFunctionFactory {
public:
    ~AvoidanceFactory() {};

    IControlFunction* FunctionCreator()  {
        return new Avoidance();
    } ;

};

#endif //OFFBOARD_AVOIDANCE_HPP
