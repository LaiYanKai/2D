cmake_minimum_required(VERSION 3.0.0)
project(TS2
    VERSION 0.1.0
    DESCRIPTION "Theta* 2D"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DP2D_DEBUG -DTS2_DEBUG")

# add_executable(TS2_exe src/main.cpp)
# target_include_directories(TS2_exe PRIVATE include)
# target_link_libraries(TS2_exe PRIVATE P2D_LIB)
# target_compile_features(TS2_exe PRIVATE cxx_std_17)
# target_compile_options(TS2_exe PRIVATE -Wall -Wextra -Wpedantic) #linux only

add_library(TS2_LIB SHARED src/TS2.cpp)
target_link_libraries(TS2_LIB PRIVATE P2D_LIB)
target_include_directories(TS2_LIB PUBLIC include)