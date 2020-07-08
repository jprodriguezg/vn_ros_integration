cmake_minimum_required(VERSION 3.0.2)
project(vn)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

add_message_files(FILES VNdata.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(vn src/vn.cpp)
add_dependencies(vn ${catkin_EXPORTED_TARGETS})
target_link_libraries(vn ${catkin_LIBRARIES})
