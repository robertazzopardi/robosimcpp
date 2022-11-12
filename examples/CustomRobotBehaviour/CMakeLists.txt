cmake_minimum_required(VERSION 3.7)
project(CustomRobotBehaviour LANGUAGES CXX)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} robosim)
