set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(main main.cpp)

target_include_directories(main PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(main PRIVATE robosim)
