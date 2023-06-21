cmake_minimum_required(VERSION 3.0.0)
project(Run2
    VERSION 0.1.0
    DESCRIPTION "Run 2D Path Planning Algorithms"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DP2D_DEBUG")

add_executable(run_exe main.cpp)
target_include_directories(run_exe PRIVATE include)
target_link_libraries(run_exe PRIVATE P2D_LIB ANYA2_LIB VG2_LIB TS2_LIB)
target_compile_features(run_exe PRIVATE cxx_std_17)
target_compile_options(run_exe PRIVATE -Wall -Wextra -Wpedantic) #linux only