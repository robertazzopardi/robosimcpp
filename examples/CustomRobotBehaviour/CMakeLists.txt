cmake_minimum_required(VERSION 3.7)
project(CustomRobotBehaviour LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(${PROJECT_NAME} main.cpp)

# Link Robosim library and headers
find_library(ROBOSIM_LIB NAMES robosim librobosim)
target_link_libraries(${PROJECT_NAME} PRIVATE SDL2 robosim)
