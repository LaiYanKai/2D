cmake_minimum_required(VERSION 3.0.0)
project(R2
    VERSION 0.1.0
    DESCRIPTION "R2"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DP2D_DEBUG -DR2_DEBUG")

# add_executable(R2_exe 
#     src/main.cpp 
#     src/node.cpp
#     src/corner.cpp
#     src/types.cpp
#     src/openlist.cpp
#     src/los.cpp
#     src/R2.cpp
#     src/query.cpp
#     src/caster.cpp
#     src/tracer.cpp
#     src/debug.cpp
#     src/checks.cpp
#     src/ray.cpp
#     src/unit.cpp
#     src/traceinfo.cpp
#     )
# target_include_directories(R2_exe PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
# target_link_libraries(R2_exe PRIVATE P2D_UTILS_LIB)
# target_compile_features(R2_exe PRIVATE cxx_std_17)
# target_compile_options(R2_exe PRIVATE -Wall -Wextra -Wpedantic) #linux only
set (SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)
add_library(R2_LIB SHARED ${SRC_DIR}/node.cpp
    ${SRC_DIR}/corner.cpp
    ${SRC_DIR}/types.cpp
    ${SRC_DIR}/R2.cpp
    ${SRC_DIR}/query.cpp
    ${SRC_DIR}/caster.cpp
    ${SRC_DIR}/tracer.cpp
    ${SRC_DIR}/checks.cpp
    ${SRC_DIR}/ray.cpp
    ${SRC_DIR}/unit.cpp
    ${SRC_DIR}/traceinfo.cpp)
target_link_libraries(R2_LIB PRIVATE P2D_LIB)
target_include_directories(R2_LIB PUBLIC include)