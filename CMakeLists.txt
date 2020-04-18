cmake_minimum_required(VERSION 3.4.1)
project(offboard)
SET(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        serial
        std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offboard
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
        include
        inc/
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/new_struct/
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/VehicleControl/
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/serial/
        ${catkin_INCLUDE_DIRS}
)

include_directories( "/usr/include/eigen3" )
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/offboard.cpp
# )
add_executable(offboard src/offboard.cpp)
target_link_libraries(offboard ${catkin_LIBRARIES})

add_executable(mserial src/serial/serialConnection.cpp)
target_link_libraries(mserial
        ${catkin_LIBRARIES})
add_dependencies(mserial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(master_serial src/serial/MasterDroneConnection.cpp)
target_link_libraries(master_serial
        ${catkin_LIBRARIES})
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

