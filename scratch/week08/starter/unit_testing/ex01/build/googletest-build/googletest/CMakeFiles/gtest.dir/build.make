# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build

# Include any dependencies generated for this target.
include googletest-build/googletest/CMakeFiles/gtest.dir/depend.make

# Include the progress variables for this target.
include googletest-build/googletest/CMakeFiles/gtest.dir/progress.make

# Include the compile flags for this target's objects.
include googletest-build/googletest/CMakeFiles/gtest.dir/flags.make

googletest-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.o: googletest-build/googletest/CMakeFiles/gtest.dir/flags.make
googletest-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.o: googletest-src/googletest/src/gtest-all.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object googletest-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.o"
	cd /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/googletest-build/googletest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gtest.dir/src/gtest-all.cc.o -c /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/googletest-src/googletest/src/gtest-all.cc

googletest-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gtest.dir/src/gtest-all.cc.i"
	cd /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/googletest-build/googletest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/googletest-src/googletest/src/gtest-all.cc > CMakeFiles/gtest.dir/src/gtest-all.cc.i

googletest-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gtest.dir/src/gtest-all.cc.s"
	cd /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/googletest-build/googletest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/googletest-src/googletest/src/gtest-all.cc -o CMakeFiles/gtest.dir/src/gtest-all.cc.s

googletest-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.requires:

.PHONY : googletest-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.requires

googletest-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.provides: googletest-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.requires
	$(MAKE) -f googletest-build/googletest/CMakeFiles/gtest.dir/build.make googletest-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.provides.build
.PHONY : googletest-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.provides

googletest-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.provides.build: googletest-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.o


# Object files for target gtest
gtest_OBJECTS = \
"CMakeFiles/gtest.dir/src/gtest-all.cc.o"

# External object files for target gtest
gtest_EXTERNAL_OBJECTS =

lib/libgtest.a: googletest-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.o
lib/libgtest.a: googletest-build/googletest/CMakeFiles/gtest.dir/build.make
lib/libgtest.a: googletest-build/googletest/CMakeFiles/gtest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../lib/libgtest.a"
	cd /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/googletest-build/googletest && $(CMAKE_COMMAND) -P CMakeFiles/gtest.dir/cmake_clean_target.cmake
	cd /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/googletest-build/googletest && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gtest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
googletest-build/googletest/CMakeFiles/gtest.dir/build: lib/libgtest.a

.PHONY : googletest-build/googletest/CMakeFiles/gtest.dir/build

googletest-build/googletest/CMakeFiles/gtest.dir/requires: googletest-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.requires

.PHONY : googletest-build/googletest/CMakeFiles/gtest.dir/requires

googletest-build/googletest/CMakeFiles/gtest.dir/clean:
	cd /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/googletest-build/googletest && $(CMAKE_COMMAND) -P CMakeFiles/gtest.dir/cmake_clean.cmake
.PHONY : googletest-build/googletest/CMakeFiles/gtest.dir/clean

googletest-build/googletest/CMakeFiles/gtest.dir/depend:
	cd /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01 /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/googletest-src/googletest /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/googletest-build/googletest /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week08/starter/unit_testing/ex01/build/googletest-build/googletest/CMakeFiles/gtest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : googletest-build/googletest/CMakeFiles/gtest.dir/depend

