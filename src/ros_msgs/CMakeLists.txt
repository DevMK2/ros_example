cmake_minimum_required(VERSION 2.8.3)
project(ros_msgs)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
	std_msgs
	message_generation
)

add_service_files(
	FILES
    Attach.srv
    Detach.srv
)

#add_message_files(
#	FILES
#   example.msg
#)

generate_messages(
	DEPENDENCIES
	std_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${std_msgs_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

catkin_package(
	CATKIN_DEPENDS message_runtime
)
