cmake_minimum_required(VERSION 3.0.0)
project(VG2
    VERSION 0.1.0
    DESCRIPTION "Visibility Graph 2D"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DP2D_DEBUG -DVG2_DEBUG")

# add_executable(VG2_exe src/main.cpp)
# target_include_directories(VG2_exe PRIVATE include)
# target_link_libraries(VG2_exe PRIVATE P2D_LIB)
# target_compile_features(VG2_exe PRIVATE cxx_std_17)
# target_compile_options(VG2_exe PRIVATE -Wall -Wextra -Wpedantic) #linux only

add_library(VG2_LIB INTERFACE)
target_include_directories(VG2_LIB INTERFACE include)