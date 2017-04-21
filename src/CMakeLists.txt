cmake_minimum_required (VERSION 3.4.3)
project (LPSolvers)

include_directories (include)
include_directories (flens)
set (SOURCES main.cpp simplex/simplex.cpp)
set (CMAKE_CXX_FLAGS_DEBUG "-Wall -pedantic -std=c++11 -ggdb -O0 -DDEBUG")
set (CMAKE_CXX_FLAGS_RELEASE "-Wall -pedantic -std=c++11 -O3")

add_executable (MYPROGRAM ${SOURCES})
