# FYI I'm still learning cmake

cmake_minimum_required(VERSION 3.0.0)
project(
  robosim
  VERSION 0.1.0
  LANGUAGES CXX)

# set everything up for c++ 20 features
set(CMAKE_CXX_STANDARD 20)
# Don't add this line if you will try_compile with boost.
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_COMPILER /usr/bin/clang++)

# Use CTest
include(CTest)
enable_testing()

# Include what you use detector
find_program(IWYU NAMES include-what-you-use)
if(IWYU)
  message(STATUS "Using include-what-you-use")
  set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${IWYU})
endif()

# Define common variables
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

set(INCLUDE_PATH ${CMAKE_SOURCE_DIR}/include/robosim)
set(SOURCE_PATH ${CMAKE_SOURCE_DIR}/src/robosim)

set(HEADER_FILES ${INCLUDE_PATH}/robosim.h ${INCLUDE_PATH}/EnvController.h
                 ${INCLUDE_PATH}/RobotMonitor.h)
set(INCLUDE_INSTALL_PATH include/robosim)
set(LIB_INSTALL_PATH lib)

# Define the sources of the library
file(GLOB SOURCES ${SOURCE_PATH}/*.h ${SOURCE_PATH}/*.cpp ${INCLUDE_PATH}/*.h)
add_library(${PROJECT_NAME} SHARED ${SOURCES})
# message(STATUS ${SOURCES})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_compile_options(
  ${PROJECT_NAME}
  PUBLIC -Werror
         -Wall
         -Wextra
         -Os
         -flto
         -std=c++20
         -pthread)

target_include_directories(${PROJECT_NAME} PUBLIC ./)
target_include_directories(${PROJECT_NAME} PUBLIC src/robosim include/robosim)

# Link with SDL2
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdl2)
# Find SDL2, SDL2_image and SDL2_gfx libraries
find_package(SDL2 REQUIRED)
# find_package(SDL2_image REQUIRED)
find_package(SDL2_gfx REQUIRED)

# Link SDL2::Main, SDL2::Image and SDL2::GFX to our project
target_link_libraries(${PROJECT_NAME} SDL2::Main SDL2::GFX)
# SDL2::Image

# Set up install commands
install(TARGETS ${PROJECT_NAME} DESTINATION ${LIB_INSTALL_PATH})
install(FILES ${HEADER_FILES} DESTINATION ${INCLUDE_INSTALL_PATH})
