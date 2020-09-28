cmake_minimum_required(VERSION 2.8.3)
project(ros_msgs)

# 빌드 패키지 의존성 추가
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
	std_msgs
	message_generation
)

# 커스텀 ROS service 메시지 파일
add_service_files(
	FILES
    Attach.srv
    Detach.srv
)

# 커스텀 ROS topic 메시지 파일
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
