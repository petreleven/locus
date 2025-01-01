cmake_minimum_required(VERSION 3.5)
project(DDesign VERSION 1.0)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD 20)
configure_file(setup.h.in setup.h)

add_executable(Main main.cpp)
add_subdirectory(locusBase)
target_include_directories(Main PUBLIC
                          "${CMAKE_BINARY_DIR}"
                          "${CMAKE_SOURCE_DIR}"
                          "${CMAKE_SOURCE_DIR}/locusBase")

target_link_libraries(Main PUBLIC MathFunctions)
target_link_libraries(Main PUBLIC Core raylib)
