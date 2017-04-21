cmake_minimum_required (VERSION 3.4.3)
project (LPSolvers)

include_directories (include)
include_directories (flens)
set (SOURCES main.cpp simplex/simplex.cpp)
set (CMAKE_CXX_FLAGS "-std=c++11 -ggdb -O0 -DDEBUG")

add_executable (MYPROGRAM ${SOURCES})
