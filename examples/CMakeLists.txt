cmake_minimum_required(VERSION 3.7)
project(
	RobosimExample
  VERSION 0.1.0
  LANGUAGES CXX)

# target_include_directories(${APP_EXE} PRIVATE ../include/robosim)

find_package(SDL2 REQUIRED)
include_directories(RobosimExample ${SDL_INCLUDE_DIRECTORIES})
find_package(SDL2_gfx REQUIRED)

include_directories(../include/robosim ../src/robosim)

FILE(GLOB SOURCES ../include/robosim/*.h ../src/robosim/*.cpp)

add_executable(RobosimExample main.cpp ${SOURCES})
target_link_libraries(RobosimExample ${SDL2_LIBRARIES} ${SDL2_GFX_LIBRARIES})
