cmake_minimum_required(VERSION 3.0.0)
project(R2S
    VERSION 0.1.0
    DESCRIPTION "R2*"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DP2D_DEBUG -DR2S_DEBUG")

# add_executable(TS2_exe src/main.cpp)
# target_include_directories(TS2_exe PRIVATE include)
# target_link_libraries(TS2_exe PRIVATE P2D_LIB)
# target_compile_features(TS2_exe PRIVATE cxx_std_17)
# target_compile_options(TS2_exe PRIVATE -Wall -Wextra -Wpedantic) #linux only

add_library(R2S_LIB INTERFACE)
target_link_libraries(R2S_LIB INTERFACE P2D_LIB)
target_include_directories(R2S_LIB INTERFACE include)