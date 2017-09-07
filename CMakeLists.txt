cmake_minimum_required(VERSION 2.8.12)

project(StereoVO)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -fopenmp" )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

include_directories( ${PROJECT_SOURCE_DIR}/include )
link_directories( ${PROJECT_SOURCE_DIR}/lib )

add_subdirectory( ${PROJECT_SOURCE_DIR}/src)
