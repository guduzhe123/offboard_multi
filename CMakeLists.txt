cmake_minimum_required(VERSION 3.4.1)
project(offboard)
SET(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        serial
        std_msgs
        message_generation
)

add_message_files(
        FILES
        DronePosUpdate.msg
)

generate_messages(DEPENDENCIES  std_msgs)

catkin_package(
        INCLUDE_DIRS inc
        LIBRARIES offboard
        CATKIN_DEPENDS
        message_runtime
        roscpp
        rospy
        std_msgs
        #DEPENDS system_lib
)

include_directories(
        inc/
        SYSTEM
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/new_struct/
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/VehicleControl/
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/serial/
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/test/1UAV_1USV/
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/test/2uav2/
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/test/2uav-usv/
        ${catkin_INCLUDE_DIRS}
)

include_directories( "/usr/include/eigen3" )
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/offboard.cpp
# )
add_executable(offboard src/offboard.cpp)
target_link_libraries(offboard ${catkin_LIBRARIES})

add_executable(mserial src/serial/serialConnection.cpp
        src/util.cpp
        )
target_link_libraries(mserial
        ${catkin_LIBRARIES}
        multi_offboard_lib)

add_dependencies(mserial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(master_serial src/serial/MasterDroneConnection.cpp)
target_link_libraries(master_serial
        ${catkin_LIBRARIES}
        multi_offboard_lib)

add_dependencies(master_serial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(multi_offboard  src/main.cpp )
target_link_libraries(multi_offboard ${catkin_LIBRARIES})

add_library(
        multi_offboard_lib

        STATIC
        src/multi_offboard.cpp
        src/PathCreator.cpp
        src/FlightManager.cpp
        src/new_struct/Avoidance.cpp
        src/new_struct/Multi_UAV_formation.cpp
        src/new_struct/Multi_USV_formation.cpp
        src/util.cpp
        src/DataMan.cpp inc/DataMan.hpp
        src/VehicleControl/MultiBoatControl.cpp
        src/VehicleControl/MultiDroneControl.cpp
        src/Calculate.cpp
        #[[src/serial/serialConnection.cpp]])

target_link_libraries(multi_offboard multi_offboard_lib)

add_executable(USVOffboard src/test/USVOffboard/main.cpp)
target_link_libraries(USVOffboard ${catkin_LIBRARIES})

#[[usv lead uav function]]
add_executable(usv_lead_uav src/test/1UAV_1USV/main.cpp)
add_library(usv_lead_uav_lib
        STATIC
        src/test/1UAV_1USV/uav_ros_Manager.cpp
        src/test/1UAV_1USV/usv_ros_Manager.cpp
        src/test/1UAV_1USV/usv_lead_uav.cpp
        src/util.cpp
        src/DataMan.cpp
        src/Calculate.cpp
        )

target_link_libraries(usv_lead_uav usv_lead_uav_lib)
target_link_libraries(usv_lead_uav ${catkin_LIBRARIES})

#[[uav lead one uav]]
add_executable(uav_lead_uav src/test/2uavs/main.cpp)
add_library(uavs_lib
        STATIC
        src/test/2uavs/uav1_ros_Manager.cpp
        src/test/2uavs/uav2_ros_Manager.cpp
        src/test/2uavs/uav_lead_uav.cpp
        src/test/2uavs/dataMan.cpp
        src/test/2uavs/avoidance.cpp
        src/util.cpp
        src/DataMan.cpp
        src/Calculate.cpp)
target_link_libraries(uav_lead_uav uavs_lib)
target_link_libraries(uav_lead_uav ${catkin_LIBRARIES})

#[[uav lead one uav and follow usv]]
add_executable(usv_lead_2uav src/test/2uavs-1usv/main.cpp)
add_library(usv_lead_2uav_lib
        STATIC
        src/test/2uavs-1usv/uav1_ros_Manager.cpp
        src/test/2uavs-1usv/uav2_ros_Manager.cpp
        src/test/2uavs-1usv/usv1_ros_Manager.cpp
        src/test/2uavs-1usv/uav_lead_uav.cpp
        src/test/2uavs-1usv/dataMan.cpp
        src/test/2uavs-1usv/avoidance.cpp
        src/util.cpp
        src/DataMan.cpp
        src/Calculate.cpp)
target_link_libraries(usv_lead_2uav usv_lead_2uav_lib)
target_link_libraries(usv_lead_2uav ${catkin_LIBRARIES})

#[[3usv control]]
add_executable(3usv src/test/3USVs/main.cpp)
add_library(3usvs_lib
        STATIC
        src/test/3USVs/usv1_ros_Manager.cpp
        src/test/3USVs/usv2_ros_Manager.cpp
        src/test/3USVs/usv3_ros_Manager.cpp
        src/test/3USVs/dataMan.cpp
        src/test/3USVs/usvs_control.cpp
        src/util.cpp
        src/DataMan.cpp
        src/Calculate.cpp
        src/PathCreator.cpp
        src/new_struct/
        src/FlightManager.cpp)
target_link_libraries(3usv 3usvs_lib multi_offboard_lib)
target_link_libraries(3usv ${catkin_LIBRARIES})

#[[4uav]]
add_executable(4uav src/test/4UAVs/main.cpp)
add_library(4uavs_lib
        STATIC
        src/test/4UAVs/uav1_ros_Manager.cpp
        src/test/4UAVs/uav2_ros_Manager.cpp
        src/test/4UAVs/uav3_ros_Manager.cpp
        src/test/4UAVs/uav4_ros_Manager.cpp
        src/util.cpp
        src/DataMan.cpp
        src/Calculate.cpp
        src/test/4UAVs/uavs_control.cpp
        src/PathCreator.cpp
        src/new_struct/)
target_link_libraries(4uav 4uavs_lib multi_offboard_lib)
target_link_libraries(4uav ${catkin_LIBRARIES})
