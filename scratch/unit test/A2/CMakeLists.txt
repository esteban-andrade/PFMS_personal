# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Change project name
project (assignment2_unit_test_Sensors) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#########################################################################
#########################################################################
# This section does not need to be edited in any way
# If you want to use the google test, simply copy this section below
# In your code and copy the CMakeLists.txt.in file which is in this directory
#
# The code will download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()
#########################################################################
#########################################################################

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


###################################
# Now the unit test bit!
# Add the unit test executable aagainst the source code of the test
add_executable(rawTests test/utest_rawdata.cpp)
# Linking to gtest
target_link_libraries(rawTests gtest gtest_main)
# Link the the library
target_link_libraries(rawTests ranger_lib)