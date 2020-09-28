cmake_minimum_required(VERSION 2.8.3)
project(model_plugin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FALGS}")

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo
  gazebo_ros
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

# 빌드를 위한 include directories  추가
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  # 사용자정의 ROS 서비스 메시지 "ros_msgs/Detach.h"를 include하기 위함
  "../../devel/include/"
)

# 빌드를 위한 link directories  추가
link_directories(${GAZEBO_LIBRARY_DIRS})

# 빌드 옵션 추가
add_compile_options(-fPIC --std=c++14 -O2 -march=native)
add_compile_options(-g)
if(${DEBUG_BUILD})
    add_compile_options(-g)
endif(${DEBUG_BUILD})
if(${PRINT_RESULTS})
    add_compile_options(-DPRINT_RESULTS)
endif(${PRINT_RESULTS})

# MagneticSphere.cc 파일를 빌드하여 lib_magnetic_sphere.so를 생성합니다
set(MAGNETIC "_magnetic_sphere")
add_library(${MAGNETIC} SHARED src/MagneticSphere.cc)
target_link_libraries(${MAGNETIC} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
