cmake_minimum_required(VERSION 3.0.2)
project(lab5)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs message_generation ${PROJECT_NAME})
find_package(Eigen3 REQUIRED)

add_message_files(
   FILES
 )

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy message_runtime
)

include_directories(
  include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
)


add_executable(lab5node src/subscriberNode.cpp)
add_dependencies(lab5node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_library(ScanMatcher src/ScanMatcher.cpp)
target_link_libraries(lab5node ScanMatcher ${catkin_LIBRARIES})
