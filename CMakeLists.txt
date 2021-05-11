cmake_minimum_required(VERSION 3.4.1)
set(CMAKE_CUDA_COMPILER "/usr/local/cuda-10.0/bin/nvcc")
project(offboard CXX CUDA)
SET(CMAKE_CXX_STANDARD 14)

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package(CUDA REQUIRED)

#set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

set(CUDA_NVCC_PLAGS ${CUDA_NVCC_PLAGS};-g;-G;-gencode;arch=sm_75;code=sm_30 -O3)
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        serial
        std_msgs
        cv_bridge
        image_transport
        message_generation
        pcl_ros
        tf
        octomap_msgs
        geometry_msgs
        message_filters
)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "CE30 OpenCV version: ${OpenCV_VERSION}, OpenCV_DIR: ${OpenCV_DIR}")

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

find_package(OMPL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED fcl)
find_path(LIBFCL_INCLUDE_DIRS fcl/config.h HINTS ${LIBFCL_INCLUDE_DIR} ${LIBFCL_INCLUDE_DIRS})
find_library(LIBFCL_LIBRARIES fcl HINTS ${LIBFCL_LIBRARY_DIRS})
#message("LIBFCL_INCLUDE_DIRS: ${LIBFCL_INCLUDE_DIRS}")
if (LIBFCL_FOUND)
    #    message("=========== FCL LIB: ${LIBFCL_LIBRARIES} ===========")
else ()
    message("=========== 请按装fcl: sudo apt install libfcl-0.5-dev ===========")
endif ()

add_message_files(
        DIRECTORY msg
        FILES
        Bspline.msg
        DronePosUpdate.msg
)

generate_messages(DEPENDENCIES  std_msgs geometry_msgs)

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
        octomap_msgs
        #DEPENDS system_lib
)

include_directories(
        inc/
        SYSTEM
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/action/
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/VehicleControl/
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/serial/
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/test/1UAV_1USV/
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/test/2uav2/
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/test/2uav-usv/
        ${catkin_INCLUDE_DIRS}
)

include_directories( "/usr/local/include/eigen3" )
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/offboard.cpp
# )


add_library(
        chlog_lib
        STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/log/ChLog.cpp
)

add_library(my_cuda
        ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_plan/poly_traj/use_cuda.cu
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/motion_plan/poly_traj/use_cuda.cuh
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/motion_plan/poly_traj/error.cuh
        )

target_link_libraries(
        my_cuda
        cublas
        cusolver
)

target_compile_features(my_cuda PUBLIC cxx_std_14)
set_target_properties( my_cuda
        PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

set_target_properties(my_cuda PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_property(TARGET my_cuda PROPERTY CUDA_ARCHITECTURES 75)

add_library(
        motion_plan_lib
        # Sets the library as a STATIC library.
        STATIC
        #---ROS or RTPS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_plan/bspline/non_uniform_bspline.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_plan/bspline_opt/bspline_optimizer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_plan/path_searching/kinodynamic_astar.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_plan/plan_env/edt_environment.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_plan/plan_env/obj_generator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_plan/plan_env/obj_predictor.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_plan/plan_env/raycast.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_plan/plan_env/sdf_map.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_plan/poly_traj/polynomial_traj.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_plan/traj_utils/planning_visualization.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_plan/mp_core/FastPathFinder.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_plan/mp_core/MPManager.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_plan/mp_core/MPPublisher.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/action/ActionMotionPlan.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/action/USV2ActionMotionPlan.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/action/USV3ActionMotionPlan.cpp
)

target_link_libraries(
        motion_plan_lib

        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${LIBFCL_LIBRARIES}
        /usr/local/lib/libnlopt.so
        ${${LINK_LIB}}
        chlog_lib
        my_cuda

)

set_property(TARGET motion_plan_lib PROPERTY CUDA_ARCHITECTURES 75)

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


add_library(
        multi_offboard_lib

        STATIC
        src/multi_offboard.cpp
        src/PathCreator.cpp
        src/FlightManager.cpp
        src/action/Avoidance.cpp
        src/action/USV_Avoidance.cpp
        src/action/Multi_UAV_formation.cpp
        src/action/Multi_USV_formation.cpp
        src/util.cpp
        src/DataMan.cpp inc/DataMan.hpp
        src/VehicleControl/MultiBoatControl.cpp
        src/VehicleControl/MultiDroneControl.cpp
        src/VehicleControl/MultiUUVControl.cpp
        src/Calculate.cpp
        src/PID.cpp
        src/action/ActionCircle.cpp
        src/PCL/PCLROSMessageManager.cpp
        #[[src/serial/serialConnection.cpp]])

#[[add_executable(multi_offboard  src/main.cpp )
target_link_libraries(multi_offboard ${catkin_LIBRARIES})

target_link_libraries(multi_offboard

        multi_offboard_lib
        chlog_lib
        motion_plan_lib
        ${OCTOMAP_LIBRARIES}
        ${LIBFCL_LIBRARIES})

add_dependencies(multi_offboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})]]


add_executable(USVOffboard src/test/USVOffboard/main.cpp)
target_link_libraries(USVOffboard ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
        ${LIBFCL_LIBRARIES})


#[[3usv control]]
add_executable(3usv src/test/3USVs/main.cpp inc/action/ActionCircle.hpp)
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
        src/action/ActionCircle.cpp
        src/action/
        src/PCL/PCLROSMessageManager.cpp
        src/FlightManager.cpp src/PCL/OctoMap.cpp)
target_link_libraries(3usv 3usvs_lib multi_offboard_lib)
target_link_libraries(3usv
        ${catkin_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
        ${LIBFCL_LIBRARIES}
        chlog_lib
        motion_plan_lib)
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
        src/action/ActionCircle.cpp
        src/test/4UAVs/uavs_control.cpp
        src/PathCreator.cpp
        src/action/ src/PCL/OctoMap.cpp)
target_link_libraries(4uav 4uavs_lib multi_offboard_lib chlog_lib motion_plan_lib)
target_link_libraries(4uavs_lib ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
        ${LIBFCL_LIBRARIES})
add_dependencies(4uav ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )


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
        src/action/ActionCircle.cpp
        src/test/3USVs/usvs_control.cpp
        src/test/4UAVs/uavs_control.cpp
        src/test/7Vehicles/7vechicles_control.cpp
        src/PathCreator.cpp
        src/PCL/PCLROSMessageManager.cpp
        src/action/ src/PCL/OctoMap.cpp)
target_link_libraries(7vehicles
        7vehicles_lib
        multi_offboard_lib
        chlog_lib
        motion_plan_lib)
target_link_libraries(7vehicles ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
        ${LIBFCL_LIBRARIES})

add_executable(CompressedToRaw src/CompressedToRaw.cpp)
TARGET_LINK_LIBRARIES(CompressedToRaw ${OpenCV_LIBS} ${catkin_LIBRARIES} )