# The minimum version of CMake Required
cmake_minimum_required (VERSION 2.6)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (Quiz_3b)

# This is added to c++11 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(sum_doubles sum_doubles.cpp)
