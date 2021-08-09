# FYI I'm still learning cmake

cmake_minimum_required(VERSION 3.0.0)
project(
  robosim
  VERSION 0.1.0
  LANGUAGES CXX)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Use CTest
include(CTest)
enable_testing()

# Include what you use detector
find_program(IWYU NAMES include-what-you-use)
if(IWYU)
  message(STATUS "executing include-what-you-use")
  set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${IWYU})
endif()

# Define common variables
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

set(INCLUDE_PATH ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME})
set(SOURCE_PATH ${CMAKE_SOURCE_DIR}/src/${PROJECT_NAME})

set(HEADER_FILES
    ${INCLUDE_PATH}/${PROJECT_NAME}.h ${INCLUDE_PATH}/EnvController.h
    ${INCLUDE_PATH}/RobotMonitor.h ${INCLUDE_PATH}/Colour.h)
set(INCLUDE_INSTALL_PATH include/${PROJECT_NAME})
set(LIB_INSTALL_PATH lib)

# Define the sources of the library
file(GLOB SOURCES ${SOURCE_PATH}/*.h ${SOURCE_PATH}/*.cpp ${INCLUDE_PATH}/*.h)
add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_compile_options(
  ${PROJECT_NAME}
  PUBLIC -Werror
         -Wall
         -Wextra
         -Oz
         -flto
         -pthread
         -ffunction-sections
         -fdata-sections)

add_custom_command(
  TARGET ${PROJECT_NAME}
  POST_BUILD
  COMMAND du -h "${CMAKE_CURRENT_SOURCE_DIR}/build/lib${PROJECT_NAME}.dylib")

target_include_directories(${PROJECT_NAME} PUBLIC ./)
target_include_directories(${PROJECT_NAME} PUBLIC src/${PROJECT_NAME}
                                                  include/${PROJECT_NAME})

set(SDL2_LIBRARY $ENV{SDL2_LIBRARY})
set(SDL2_INCLUDE_DIR $ENV{SDL2_INCLUDE_DIR})

# Link with SDL2
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdl2)

# Find SDL2 and SDL2_gfx libraries
find_package(SDL2 REQUIRED)
find_package(SDL2_gfx REQUIRED)

# Link SDL2::Main and SDL2::GFX to our project
target_link_libraries(${PROJECT_NAME} SDL2::Main SDL2::GFX)

# Set up install commands
install(TARGETS ${PROJECT_NAME} DESTINATION ${LIB_INSTALL_PATH})
install(FILES ${HEADER_FILES} DESTINATION ${INCLUDE_INSTALL_PATH})
