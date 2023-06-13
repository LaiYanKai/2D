cmake_minimum_required(VERSION 3.0.0)
project(ANYA2
    VERSION 0.1.0
    DESCRIPTION "ANYA 2D"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DP2D_DEBUG -DANYA2_DEBUG")

add_executable(ANYA2_exe src/main.cpp src/debug.cpp)
target_include_directories(ANYA2_exe PRIVATE include)
target_link_libraries(ANYA2_exe PRIVATE P2D_LIB)
target_compile_features(ANYA2_exe PRIVATE cxx_std_17)
target_compile_options(ANYA2_exe PRIVATE -Wall -Wextra -Wpedantic) #linux only