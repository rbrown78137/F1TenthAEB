cmake_minimum_required(VERSION 3.0.2)
project(aeb)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs message_generation)


add_message_files(
   FILES
 )
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )
catkin_package(
    CATKIN_DEPENDS roscpp rospy message_runtime
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(aeb src/aeb.cpp)
add_dependencies(aeb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aeb ${catkin_LIBRARIES})
