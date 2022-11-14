cmake_minimum_required(VERSION 3.7)
project(CustomRobotBehaviour LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(${PROJECT_NAME} main.cpp)

# Link Robosim library and headers
find_library(ROBOSIM_LIB NAMES robosim librobosim)
target_link_libraries(${PROJECT_NAME} PRIVATE ${ROBOSIM_LIB})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_INSTALL_PREFIX}/include)
