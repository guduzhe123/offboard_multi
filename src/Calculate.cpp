//
// Created by zhouhua on 2020/1/28.
//

#include "Calculate.hpp"
Calculate* Calculate::l_pInst = NULL;

Calculate::Calculate() {

}

void Calculate::GetLocalPos(const GlobalPosition &loc1, const GlobalPosition &loc2,
                            TVec3 &follow_uav_local_pos) {

    float x, y;
    get_vector_to_next_waypoint_fast(loc1.latitude, loc1.longitude, loc2.latitude, loc2.longitude, x, y);
    util_log("takeoff lat = %.8f, lon = %.8f, way lat = %.8f, way lon = %.8f", loc1.latitude, loc1.longitude, loc2.latitude, loc2.longitude);

    follow_uav_local_pos.x() = -y;
    follow_uav_local_pos.y() = -x;
    util_log("distance orig x= %.2f, distance orig y= %.2f, x new = %.2f, y new = %.2f",
            follow_uav_local_pos.x(), follow_uav_local_pos.y(), x, y);
}

void Calculate::get_vector_to_next_waypoint_fast(double lat_now, double lon_now, double lat_next, double lon_next,
                                               float &v_n, float &v_e)
{
    double lat_now_rad = lat_now * M_DEG_TO_RAD;
    double lon_now_rad = lon_now * M_DEG_TO_RAD;
    double lat_next_rad = lat_next * M_DEG_TO_RAD;
    double lon_next_rad = lon_next * M_DEG_TO_RAD;

    double d_lat = lat_next_rad - lat_now_rad;
    double d_lon = lon_next_rad - lon_now_rad;

    /* conscious mix of double and float trig function to maximize speed and efficiency */
    v_n = CONSTANTS_RADIUS_OF_EARTH * d_lat;
    v_e = CONSTANTS_RADIUS_OF_EARTH * d_lon * cos(lat_now_rad);
}

void Calculate::GetGlobalPos(const GlobalPosition &loc1, GlobalPosition &loc2, TVec2 &local_target_pos) {
    GlobalPosition center_pos;
    double meterPerLat, meterPerLongt;
    double x,y;
    x = local_target_pos(0);
    y = local_target_pos(1);

    center_pos.latitude = 0;
    center_pos.longitude = 0;
    getMeterScaleHere(meterPerLat, meterPerLongt, center_pos);

    double dLongt, dLat;
    float  k = 2.0f/3.0f; // TODO PX4 x axis data is large 1.5
    dLat = y / meterPerLat;
    dLongt = x /(meterPerLongt * k); // ?

    loc2.longitude = loc1.longitude + dLongt;
    loc2.latitude = loc1.latitude + dLat;
}

double Calculate::calcDist(const GlobalPosition &loc1, const GlobalPosition &loc2)
{
    double R = 6371; // Radius of the earth in km
    double dLat = deg2rad(loc2.latitude - loc1.latitude);  // deg2rad below
    double dLon = deg2rad(loc2.longitude - loc1.longitude);
    double a =
            sin(dLat / 2) * sin(dLat / 2) +
            cos(deg2rad(loc1.latitude)) * cos(deg2rad(loc2.longitude)) *
            sin(dLon / 2) * sin(dLon / 2)
    ;
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double d = R * c; // Distance in km
    return d*1000.0;	// from km to meters
}

double Calculate::deg2rad(double deg) {
    return deg * (M_PI / 180.0);
}

double Calculate::rad2deg(double rad) {
    return rad * (180.0 / M_PI);
}

float Calculate::dgrIn180s(float d) {
    int n = d / 360;
    float d1 = d - (float) (n * 360);
    if (d1 > 180) d1 -= 360;
    if (d1 < -180)d1 += 360;
    return d1;
}

void Calculate::getMeterScaleHere(double &meterPerLatUnit, double &meterPerLongtUnit,
                                       const GlobalPosition &center_pos)
{
    {
        GlobalPosition loc0 = center_pos;
        GlobalPosition loc1 = center_pos;

        loc0.latitude -= 0.5;
        loc1.latitude += 0.5;
        meterPerLatUnit = fabs(calcDist(loc0, loc1));

    }

    {
        GlobalPosition loc0 = center_pos;
        GlobalPosition loc1 = center_pos;

        loc0.longitude -= 0.5;
        loc1.longitude += 0.5;
        meterPerLongtUnit = fabs(calcDist(loc0, loc1));

    }

}

float Calculate::get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
    double lat_now_rad = lat_now * M_DEG_TO_RAD;
    double lon_now_rad = lon_now * M_DEG_TO_RAD;
    double lat_next_rad = lat_next * M_DEG_TO_RAD;
    double lon_next_rad = lon_next * M_DEG_TO_RAD;

    double d_lon = lon_next_rad - lon_now_rad;

    /* conscious mix of double and float trig function to maximize speed and efficiency */
    float theta = atan2f(sin(d_lon) * cos(lat_next_rad),
                         cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos(lat_next_rad) * cos(d_lon));

    theta = wrap_pi(theta);

    return theta;
}

float Calculate::wrap_pi(float bearing)
{
    /* value is inf or NaN */
    if (!isfinite(bearing)) {
        return bearing;
    }

    int c = 0;

    while (bearing >= M_PI) {
        bearing -= 2* M_PI;

        if (c++ > 3) {
            return NAN;
        }
    }

    c = 0;

    while (bearing < -M_PI) {
        bearing += 2 * M_PI;

        if (c++ > 3) {
            return NAN;
        }
    }

    return bearing;
}

void Calculate::quaternion_to_rpy(geometry_msgs::Quaternion orientation, double &roll, double &pitch, double &yaw) {
    /*Frame	|                     ROS	                                       | PX4
    Body	|FLU (X Forward, Y Left, Z Up), usually named base_link	           |FRD (X Forward, Y Right and Z Down)
    World	|ENU (X East, Y North and Z Up), with the naming being odom or map |NED (X North, Y East, Z Down)
     */
    tf::Quaternion RQ2;
    tf::quaternionMsgToTF(orientation, RQ2);
    tf::Matrix3x3(RQ2).getRPY(roll, pitch, yaw);
}

Eigen::Quaterniond Calculate::quaternion_from_rpy(const double roll, const double pitch, const double yaw) {
    return quaternion_from_rpy(Eigen::Vector3d(roll, pitch, yaw));
}
/*
 * Note: order of axis are match tf2::LinearMath (bullet).
 * YPR rotation convention -> YAW first, Pitch second, Roll third
 * Compatibility checked by unittests.
 */

Eigen::Quaterniond Calculate::quaternion_from_rpy(const Eigen::Vector3d &rpy)
{
    // YPR - ZYX
    return Eigen::Quaterniond(
            Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
    );
}

Eigen::Quaterniond Calculate::transform_orientation(const Eigen::Quaterniond &q, const StaticTF transform)
{
    // Transform the attitude representation from frame to frame.
    // The proof for this transform can be seen
    // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/
    static const auto NED_ENU_Q = quaternion_from_rpy(M_PI, 0.0, M_PI_2);
    switch (transform) {
        case StaticTF::NED_TO_ENU:
        case StaticTF::ENU_TO_NED:
            return NED_ENU_Q * q;
    }
}

double Calculate::quaternion_get_yaw(const geometry_msgs::Quaternion &orientation, EulerAngles& angle)
{
    // to match equation from:
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    Eigen::Quaterniond q, Q;
    Q.w() = orientation.w;
    Q.x() = orientation.x;
    Q.y() = orientation.y;
    Q.z() = orientation.z;
    q = transform_orientation(Q, StaticTF::ENU_TO_NED);
//    q = Q;
    const double &q0 = q.w();
    const double &q1 = q.x();
    const double &q2 = q.y();
    const double &q3 = q.z();


    return std::atan2(2. * (q0*q3 + q1*q2), 1. - 2. * (q2*q2 + q3*q3));
}

void Calculate::posToPosCtrl(TVec3 &target_point, TVec3 &target_after_judge, TVec3 &drone_cur_pos, float speed_limit) {
    // get orign target closer to drone point
    TVec3 pos_offset = target_point - drone_cur_pos;

    if (speed_limit < 0.1f) {
        speed_limit = m_speedLimit;
    }
    speed_limit *= 2;

    //防止 一个向量过大 导致另外一个坐标轴的误差不被修正
    //limit x/y/z to speed_limit, to avoid(5,0,100), one dimension is far smaller than another one
    //3/15=0.2
    if (pos_offset.norm() > 20) {
        if (fabsf(pos_offset.x()) > speed_limit) {
            pos_offset.x() = (pos_offset.x() > 0) ? speed_limit : -speed_limit;
        }
        if (fabsf(pos_offset.y()) > speed_limit) {
            pos_offset.y() = (pos_offset.y() > 0) ? speed_limit : -speed_limit;
        }
        if (fabsf(pos_offset.z()) > speed_limit) {
            pos_offset.z() = (pos_offset.z() > 0) ? speed_limit : -speed_limit;
        }
    }

    float pos_len = pos_offset.norm();
    if (pos_len > speed_limit) {
        pos_offset = speed_limit * pos_offset.normalized();
    }
    target_after_judge = pos_offset + drone_cur_pos;
}

void Calculate::getTakeoffPos(M_Drone &master, M_Drone &slave, TVec3 &follow_slave_first_local) {
    GlobalPosition master_start_gps, slave_takeoff_gps;

    if (slave.current_state.armed && !is_get_takeoff_pos_) {
        master_start_gps = GlobalPosition{master.latitude, master.longtitude, 0};
        slave_takeoff_gps = GlobalPosition{slave.latitude, slave.longtitude, 0};
        util_log("master_start_gps_ = ( %.9f, %.9f)", master_start_gps.latitude, master_start_gps.longitude);
        util_log("slave_takeoff_gps_ = ( %.9f, %.9f)", slave_takeoff_gps.latitude, slave_takeoff_gps.longitude);

        GetLocalPos(master_start_gps, slave_takeoff_gps, follow_slave_first_local);
        util_log("follow_slave_first_local = ( %.2f, %.2f, %.2f)", follow_slave_first_local.x(), follow_slave_first_local.y(),
                 follow_slave_first_local.z());

        is_get_takeoff_pos_ = true;
    }
}


void Calculate::bodyFrame2LocalFrame(geometry_msgs::PoseStamped &body, geometry_msgs::PoseStamped &local, float yaw) {
    local.pose.position.x = body.pose.position.x * cos(yaw) - body.pose.position.y * sin(yaw);
    local.pose.position.y = body.pose.position.x * sin(yaw) + body.pose.position.y * cos(yaw);
    local.pose.position.z = body.pose.position.z;
    util_log("body to local cos(%.2f) = %.2f, sin(%.2f) = %.2f" , yaw, cos(yaw), yaw, sin(yaw));
}


void Calculate::localFrame2BodyFrame(geometry_msgs::PoseStamped &local, geometry_msgs::PoseStamped &body, float yaw) {
    body.pose.position.x = local.pose.position.x * cos(yaw) + local.pose.position.y * sin(yaw);
    body.pose.position.y = -local.pose.position.x * sin(yaw) + local.pose.position.y * cos(yaw);
    body.pose.position.z = local.pose.position.z;
    util_log("local to body cos(%.2f) = %.2f, sin(%.2f) = %.2f" , yaw, cos(yaw), yaw, sin(yaw));
}

//rad!!! 注意这个接口是NED，不能和EUS直接相乘！！！
TQuat Calculate::EulerAngle2QuatNED(const float ned_yaw, const float ned_pitch, const float ned_roll) {
    //auto rot = mavros_ftf::quaternion_from_rpy(ned_yaw, ned_pitch, ned_roll);
    Eigen::AngleAxisf roll(Eigen::AngleAxisf(ned_roll, TVec3::UnitX()));
    Eigen::AngleAxisf pitch(Eigen::AngleAxisf(ned_pitch, TVec3::UnitY()));
    Eigen::AngleAxisf yaw(Eigen::AngleAxisf(ned_yaw, TVec3::UnitZ()));
    TQuat q = yaw * pitch * roll;
    return q;//!!!
}

TVec3 Calculate::toVec(const float r, const float p, const float h) {
    TVec3 v;
    v.x() = cos(h);
    v.y() = sin(h);
    v.z() = 1;
    return v;
}

Calculate* Calculate::getInstance() {
    if (l_pInst == NULL) {
        l_pInst = new Calculate();
    }
    return l_pInst;
}