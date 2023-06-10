cmake_minimum_required(VERSION 3.2.2)
project(P2D
    VERSION 1.0.0
    DESCRIPTION "P2D"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DP2D_DEBUG")

add_library(P2D_LIB INTERFACE)
target_include_directories(P2D_LIB INTERFACE include)

# add_executable(P2D_exe src/main.cpp) #test
# target_include_directories(P2D_exe PRIVATE include)
# target_compile_features(P2D_exe PRIVATE cxx_std_17)
# target_compile_options(P2D_exe PRIVATE -Wall -Wextra -Wpedantic) #linux only