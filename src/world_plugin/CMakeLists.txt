cmake_minimum_required(VERSION 2.8.3)
project(world_plugin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FALGS}")

# 빌드 패키지 의존성 추가
find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  gazebo
  gazebo_ros
)

# 빌드를 위한 include directories  추가
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# 빌드를 위한 link directories  추가
link_directories(${GAZEBO_LIBRARY_DIRS})

# 빌드 옵션 추가
add_compile_options(-fPIC --std=c++14 -O2 -march=native)
if(${DEBUG_BUILD})
    add_compile_options(-g)
endif(${DEBUG_BUILD})
if(${PRINT_RESULTS})
    add_compile_options(-DPRINT_RESULTS)
endif(${PRINT_RESULTS})

# SimulationWorld.cc 파일를 빌드하여 lib_simulation_world.so를 생성합니다
set(SIMULATION_WORLD "_simulation_world")
add_library(${SIMULATION_WORLD} SHARED src/SimulationWorld.cc)
target_link_libraries(${SIMULATION_WORLD} ${GAZEBO_LIBRARIES}) 
