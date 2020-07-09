cmake_minimum_required(VERSION 3.0.2)
project(vn)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs)

include_directories(include/vn)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(vn src/vn_node.cpp src/VNdata.cpp src/UARTcom.cpp)
add_dependencies(vn ${catkin_EXPORTED_TARGETS})
target_link_libraries(vn ${catkin_LIBRARIES})
