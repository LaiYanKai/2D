cmake_minimum_required(VERSION 3.2.2)
project(RR2star
    VERSION 1.0.0
    DESCRIPTION "RR2star"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DRR2STAR_DEBUG")

add_executable(RR2star_exe 
    src/main.cpp
)
target_include_directories(RR2star_exe PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
target_compile_features(RR2star_exe PRIVATE cxx_std_17)
target_compile_options(RR2star_exe PRIVATE -Wall -Wextra -Wpedantic) #linux only