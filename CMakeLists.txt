cmake_minimum_required(VERSION 3.7.0)
project(
  robosim
  VERSION 0.1.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Use CTest
include(CTest)
enable_testing()

set(INCLUDE_PATH ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME})
set(SOURCE_PATH ${CMAKE_SOURCE_DIR}/src/${PROJECT_NAME})

set(HEADER_FILES
    ${INCLUDE_PATH}/${PROJECT_NAME}.h 
    ${INCLUDE_PATH}/EnvController.h
    ${INCLUDE_PATH}/RobotMonitor.h 
    ${INCLUDE_PATH}/Colour.h)

set(INCLUDE_INSTALL_PATH include/${PROJECT_NAME})
set(LIB_INSTALL_PATH lib)

# Define the sources of the library
file(GLOB SOURCES
    ${SOURCE_PATH}/*.h 
    ${SOURCE_PATH}/*.cpp 
    ${INCLUDE_PATH}/*.h)
add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_compile_options(
  ${PROJECT_NAME}
  PUBLIC -Werror
         -Wall
         -Wextra
         -O3
         -flto
         -pthread
         -ffunction-sections
         -fdata-sections)

target_include_directories(${PROJECT_NAME} PUBLIC src/${PROJECT_NAME}
                                                  include/${PROJECT_NAME})

# Link with SDL2
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdl2)
find_package(SDL2 REQUIRED)
find_package(SDL2_gfx REQUIRED)
target_link_libraries(${PROJECT_NAME} SDL2::Main SDL2::GFX)

# Set up install commands
install(TARGETS ${PROJECT_NAME} DESTINATION ${LIB_INSTALL_PATH})
install(FILES ${HEADER_FILES} DESTINATION ${INCLUDE_INSTALL_PATH})
