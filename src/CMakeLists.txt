cmake_minimum_required (VERSION 3.4.3)
project (LPSolvers)

include_directories (include)
include_directories (flens)
set (SOURCES main.cpp simplex/simplex.cpp utils/mpsparser.cpp utils/utils.cpp ipm/ipm.cpp lp/lp.cpp)
set (CMAKE_CXX_FLAGS "")
set (CMAKE_CXX_FLAGS_DEBUG "-std=c++14 -ggdb -O0 -DDEBUG")
set (CMAKE_CXX_FLAGS_RELEASE "-std=c++14 -O0")

add_executable (MYPROGRAM ${SOURCES})
