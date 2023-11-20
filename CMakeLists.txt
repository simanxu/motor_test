cmake_minimum_required(VERSION 3.5)

project(MotorTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(BUILD_TYPE_RELEASE TRUE)
add_definitions(-w)

add_subdirectory(beigo_test)
add_subdirectory(cyber_test)
