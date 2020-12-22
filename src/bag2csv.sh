#!/usr/bin/env bash

#rostopic echo -b multi_data_2020-10-25-23-00-45_0.bag -p /usv1/mavros/debug_value/debug_vector > usv1_command.csv
#rostopic echo -b multi_data_2020-10-25-23-00-45_0.bag -p /usv1/mavros/local_position/pose > usv1_local_pos.csv
#rostopic echo -b multi_data_2020-10-25-23-00-45_0.bag -p /usv1/mavros/global_position/local > usv1_vel.csv
rostopic echo -b multi_data_2020-10-25-23-00-45_0.bag -p /usv1/mavros/vfr_hud > usv1_throttle.csv
#rostopic echo -b multi_data_2020-10-25-23-00-45_0.bag -p /usv1/mavros/setpoint_position/local > usv1_target_point.csv

#rostopic echo -b multi_data_2020-10-25-23-00-45_0.bag -p /usv2/mavros/debug_value/debug_vector > usv2_command.csv
#rostopic echo -b multi_data_2020-10-25-23-00-45_0.bag -p /usv2/mavros/local_position/pose > usv2_local_pos.csv
#rostopic echo -b multi_data_2020-10-25-23-00-45_0.bag -p /usv2/mavros/global_position/local > usv2_vel.csv
rostopic echo -b multi_data_2020-10-25-23-00-45_0.bag -p /usv2/mavros/vfr_hud > usv2_throttle.csv
#rostopic echo -b multi_data_2020-10-25-23-00-45_0.bag -p /usv2/mavros/setpoint_position/local > usv2_target_point.csv

#rostopic echo -b multi_data_2020-10-25-23-00-45_0.bag -p /usv3/mavros/debug_value/debug_vector > usv3_command.csv
#rostopic echo -b multi_data_2020-10-25-23-00-45_0.bag -p /usv3/mavros/local_position/pose > usv3_local_pos.csv
#rostopic echo -b multi_data_2020-10-25-23-00-45_0.bag -p /usv3/mavros/global_position/local > usv3_vel.csv
rostopic echo -b multi_data_2020-10-25-23-00-45_0.bag -p /usv3/mavros/vfr_hud > usv3_throttle.csv
#rostopic echo -b multi_data_2020-10-25-23-00-45_0.bag -p /usv3/mavros/setpoint_position/local > usv3_target_point.csv