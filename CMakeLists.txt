cmake_minimum_required(VERSION 2.8.3)
project(move_toward)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    serial
    std_msgs
    geometry_msgs
    tf
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(move_toward src/move_toward.cpp)
target_link_libraries(move_toward ${catkin_LIBRARIES})
