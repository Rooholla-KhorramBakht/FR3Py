cmake_minimum_required(VERSION 3.10.2)
project(fr3py)

# Using C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# In order to quiet the find_package(dynamic-graph[-python] REQUIRED)
set(CHECK_MINIMAL_CXX_STANDARD ON)
set(Python_ADDITIONAL_VERSIONS 3.6)


# dependencies 
find_package(lcm REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_subdirectory(src)