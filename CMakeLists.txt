cmake_minimum_required(VERSION 3.4.1)
project(offboard)
SET(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        serial
        std_msgs
        cv_bridge
        image_transport
        message_generation
)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "CE30 OpenCV version: ${OpenCV_VERSION}, OpenCV_DIR: ${OpenCV_DIR}")

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
        cv_bridge
        image_transport
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
        src/new_struct/USV_Avoidance.cpp
        src/new_struct/Multi_UAV_formation.cpp
        src/new_struct/Multi_USV_formation.cpp
        src/util.cpp
        src/DataMan.cpp inc/DataMan.hpp
        src/VehicleControl/MultiBoatControl.cpp
        src/VehicleControl/MultiDroneControl.cpp
        src/VehicleControl/MultiUUVControl.cpp
        src/Calculate.cpp
        src/PID.cpp
        src/ActionCircle.cpp
        #[[src/serial/serialConnection.cpp]])

target_link_libraries(multi_offboard multi_offboard_lib)

add_executable(USVOffboard src/test/USVOffboard/main.cpp)
target_link_libraries(USVOffboard ${catkin_LIBRARIES})


#[[3usv control]]
add_executable(3usv src/test/3USVs/main.cpp inc/ActionCircle.hpp)
add_library(3usvs_lib
        STATIC
        src/test/3USVs/usv1_ros_Manager.cpp
        src/test/3USVs/usv2_ros_Manager.cpp
        src/test/3USVs/usv3_ros_Manager.cpp
        src/test/UUV_Control/uuv1_ros_Manager.cpp
        src/test/3USVs/dataMan.cpp
        src/test/3USVs/usvs_control.cpp
        src/util.cpp
        src/DataMan.cpp
        src/Calculate.cpp
        src/PathCreator.cpp
        src/PID.cpp
        src/ActionCircle.cpp
        src/new_struct/
        src/FlightManager.cpp)
target_link_libraries(3usv 3usvs_lib multi_offboard_lib)
target_link_libraries(3usv ${catkin_LIBRARIES})
add_dependencies(3usv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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
        src/PID.cpp
        src/ActionCircle.cpp
        src/test/4UAVs/uavs_control.cpp
        src/PathCreator.cpp
        src/new_struct/)
target_link_libraries(4uav 4uavs_lib multi_offboard_lib)
target_link_libraries(4uavs_lib ${catkin_LIBRARIES})
add_dependencies(4uav ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#[[4uav]]
add_executable(7vehicles src/test/7Vehicles/main.cpp)
add_library(7vehicles_lib
        STATIC
        src/test/3USVs/usv1_ros_Manager.cpp
        src/test/3USVs/usv2_ros_Manager.cpp
        src/test/3USVs/usv3_ros_Manager.cpp
        src/test/UUV_Control/uuv1_ros_Manager.cpp
        src/test/4UAVs/uav1_ros_Manager.cpp
        src/test/4UAVs/uav2_ros_Manager.cpp
        src/test/4UAVs/uav3_ros_Manager.cpp
        src/test/4UAVs/uav4_ros_Manager.cpp
        src/util.cpp
        src/DataMan.cpp
        src/Calculate.cpp
        src/PID.cpp
        src/ActionCircle.cpp
        src/test/3USVs/usvs_control.cpp
        src/test/4UAVs/uavs_control.cpp
        src/test/7Vehicles/7vechicles_control.cpp
        src/PathCreator.cpp
        src/new_struct/)
target_link_libraries(7vehicles 7vehicles_lib multi_offboard_lib)
target_link_libraries(7vehicles ${catkin_LIBRARIES})

add_executable(CompressedToRaw src/CompressedToRaw.cpp)
TARGET_LINK_LIBRARIES(CompressedToRaw ${OpenCV_LIBS} ${catkin_LIBRARIES})