cmake_minimum_required(VERSION 3.10)
project(gripper)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Franka 0.10.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

add_subdirectory(external/pybind11)

pybind11_add_module(gripper MODULE src/gripper.cpp)
target_link_libraries(gripper PRIVATE ${Franka_LIBRARIES})

install(TARGETS gripper DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install)