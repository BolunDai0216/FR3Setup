cmake_minimum_required(VERSION 3.10)
project(read_fr3_state)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Franka 0.10.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

add_subdirectory(external/pybind11)
include_directories(${CMAKE_SOURCE_DIR}/include)

pybind11_add_module(read_fr3_state MODULE src/read_fr3_state.cpp)
target_link_libraries(read_fr3_state PRIVATE ${Franka_LIBRARIES})

install(TARGETS read_fr3_state DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install)