cmake_minimum_required(VERSION 3.5)

project(MotorTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(BUILD_TYPE_RELEASE TRUE)
add_definitions(-w)

find_library(SOCKETCAN_PATH socketcan)

add_executable(motor_ctrl
motor_control.cpp
)
target_include_directories(motor_ctrl
PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(motor_ctrl
PUBLIC
  rt
  pthread
  ${SOCKETCAN_PATH}
)
target_compile_options(motor_ctrl PRIVATE "-O3")
set_target_properties(motor_ctrl PROPERTIES POSITION_INDEPENDENT_CODE ON)
