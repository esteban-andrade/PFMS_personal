# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Change project name
project (assignment2) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
add_custom_target(doc
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
)
endif(DOXYGEN_FOUND)

# CMake will create your library, add the files needed
#add_library(ranger_lib ranger.cpp  rangerinterface.cpp laser.cpp sonar.cpp) add STATIC
add_library(ranger_lib STATIC ranger.cpp laser.cpp sonar.cpp rangerfusion.cpp) 

# CMake will create the cell library that you will need 
add_library(cell cell.cpp)

# CMake will create executable add the files needed
add_executable(assignment2 main.cpp)



# Cmake will link against the libraries in final step
target_link_libraries(assignment2 ranger_lib cell)
