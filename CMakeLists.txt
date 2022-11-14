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

# Define the sources of the library
file(GLOB SOURCES
    ${CMAKE_SOURCE_DIR}/src/robosim/*.h
    ${CMAKE_SOURCE_DIR}/src/robosim/*.cpp
    ${CMAKE_SOURCE_DIR}/include/robosim/*.h)

add_library(robosim SHARED ${SOURCES})

target_compile_options(
    robosim
    PRIVATE -Werror
    -Wall
    -Wextra
    -O3
    -flto=auto
    -pthread
    -ffunction-sections
    -fdata-sections)

target_include_directories(robosim PUBLIC src/robosim include/robosim)

# Link with SDL2
if (APPLE)
    set(SDL2_LIBRARY /opt/homebrew/opt/sdl2/lib/libsdl2.dylib)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdl2)
find_package(SDL2 REQUIRED)
find_package(SDL2_gfx REQUIRED)
target_link_libraries(robosim PRIVATE SDL2::Main SDL2::GFX)

# Set up install commands
install(TARGETS robosim DESTINATION lib)
set(HEADER_FILES
    ${CMAKE_SOURCE_DIR}/include/robosim/robosim.h
    ${CMAKE_SOURCE_DIR}/include/robosim/EnvController.h
    ${CMAKE_SOURCE_DIR}/include/robosim/RobotMonitor.h
    ${CMAKE_SOURCE_DIR}/include/robosim/Colour.h)
install(FILES ${HEADER_FILES} DESTINATION include/robosim)
