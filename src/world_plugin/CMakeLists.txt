cmake_minimum_required(VERSION 2.8.3)
project(world_plugin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FALGS}")

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  gazebo
  gazebo_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_compile_options(-fPIC --std=c++14 -O2 -march=native)
if(${DEBUG_BUILD})
    add_compile_options(-g)
endif(${DEBUG_BUILD})
if(${PRINT_RESULTS})
    add_compile_options(-DPRINT_RESULTS)
endif(${PRINT_RESULTS})

set(SIMULATION_WORLD "_simulation_world")
add_library(${SIMULATION_WORLD} SHARED src/SimulationWorld.cc)
target_link_libraries(${SIMULATION_WORLD} ${GAZEBO_LIBRARIES}) 
