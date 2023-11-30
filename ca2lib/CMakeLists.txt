cmake_minimum_required(VERSION 3.16)

project(ca2lib)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# set(CMAKE_BUILD_TYPE Release)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    sensor_msgs
    message_filters)

catkin_package(
    INCLUDE_DIRS
    "include/"
    LIBRARIES
    ca2lib
    CATKIN_DEPENDS
    roscpp
    cv_bridge
    sensor_msgs
    message_filters
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
message("${PROJECT_NAME}|compiling with these CXX flags: ${CMAKE_CXX_FLAGS}")

add_library(ca2lib
    src/targets/target_base.cpp
    src/targets/target_checkerboard.cpp
    src/targets/target_charuco.cpp
    src/io/ros.cpp
    src/plane_extractor/plane_extractor_monocular.cpp
    src/plane_extractor/plane_extractor_lidar.cpp
    src/projection.cpp
    src/solver/solver.cpp
    src/parse_command_line.cpp
    src/env.cpp)

target_include_directories(ca2lib PUBLIC include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(ca2lib PUBLIC ${OpenCV_LIBRARIES} Eigen3::Eigen ${catkin_LIBRARIES} yaml-cpp)

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_target tests/test_target.cpp)
    target_link_libraries(test_target ca2lib ${catkin_LIBRARIES})

    catkin_add_gtest(test_extractor tests/test_extractor.cpp)
    target_link_libraries(test_extractor ca2lib ${catkin_LIBRARIES})

    catkin_add_gtest(test_solver tests/test_solver.cpp)
    target_link_libraries(test_solver ca2lib ${catkin_LIBRARIES})

    catkin_add_gtest(test_geometry tests/test_geometry.cpp)
    target_link_libraries(test_geometry ca2lib ${catkin_LIBRARIES})
endif()

add_executable(cloud_projection_example bin/cloud_projection_example.cpp)
target_link_libraries(cloud_projection_example PRIVATE ca2lib)

add_executable(cloud_plane_estimator bin/cloud_plane_estimator.cpp)
target_link_libraries(cloud_plane_estimator PRIVATE ca2lib)

add_executable(cloud_plane_estimator_interactive bin/cloud_plane_estimator_interactive.cpp)
target_link_libraries(cloud_plane_estimator_interactive PRIVATE ca2lib)

add_executable(stereo_recorder bin/stereo_recorder.cpp)
target_link_libraries(stereo_recorder PRIVATE ca2lib spdlog)

add_executable(compare_projections bin/compare_projections.cpp)
target_link_libraries(compare_projections PRIVATE ca2lib spdlog)
add_executable(solver_camera_lidar bin/solver_camera_lidar.cpp)
target_link_libraries(solver_camera_lidar PRIVATE ca2lib spdlog)
