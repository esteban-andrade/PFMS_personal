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
CMAKE_SOURCE_DIR = /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/build

# Include any dependencies generated for this target.
include CMakeFiles/radar.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/radar.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/radar.dir/flags.make

CMakeFiles/radar.dir/main.cpp.o: CMakeFiles/radar.dir/flags.make
CMakeFiles/radar.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/radar.dir/main.cpp.o"
	/usr/bin/g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/radar.dir/main.cpp.o -c /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/main.cpp

CMakeFiles/radar.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/radar.dir/main.cpp.i"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/main.cpp > CMakeFiles/radar.dir/main.cpp.i

CMakeFiles/radar.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/radar.dir/main.cpp.s"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/main.cpp -o CMakeFiles/radar.dir/main.cpp.s

CMakeFiles/radar.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/radar.dir/main.cpp.o.requires

CMakeFiles/radar.dir/main.cpp.o.provides: CMakeFiles/radar.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/radar.dir/build.make CMakeFiles/radar.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/radar.dir/main.cpp.o.provides

CMakeFiles/radar.dir/main.cpp.o.provides.build: CMakeFiles/radar.dir/main.cpp.o


CMakeFiles/radar.dir/radar.cpp.o: CMakeFiles/radar.dir/flags.make
CMakeFiles/radar.dir/radar.cpp.o: ../radar.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/radar.dir/radar.cpp.o"
	/usr/bin/g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/radar.dir/radar.cpp.o -c /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/radar.cpp

CMakeFiles/radar.dir/radar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/radar.dir/radar.cpp.i"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/radar.cpp > CMakeFiles/radar.dir/radar.cpp.i

CMakeFiles/radar.dir/radar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/radar.dir/radar.cpp.s"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/radar.cpp -o CMakeFiles/radar.dir/radar.cpp.s

CMakeFiles/radar.dir/radar.cpp.o.requires:

.PHONY : CMakeFiles/radar.dir/radar.cpp.o.requires

CMakeFiles/radar.dir/radar.cpp.o.provides: CMakeFiles/radar.dir/radar.cpp.o.requires
	$(MAKE) -f CMakeFiles/radar.dir/build.make CMakeFiles/radar.dir/radar.cpp.o.provides.build
.PHONY : CMakeFiles/radar.dir/radar.cpp.o.provides

CMakeFiles/radar.dir/radar.cpp.o.provides.build: CMakeFiles/radar.dir/radar.cpp.o


CMakeFiles/radar.dir/dataprocessing.cpp.o: CMakeFiles/radar.dir/flags.make
CMakeFiles/radar.dir/dataprocessing.cpp.o: ../dataprocessing.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/radar.dir/dataprocessing.cpp.o"
	/usr/bin/g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/radar.dir/dataprocessing.cpp.o -c /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/dataprocessing.cpp

CMakeFiles/radar.dir/dataprocessing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/radar.dir/dataprocessing.cpp.i"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/dataprocessing.cpp > CMakeFiles/radar.dir/dataprocessing.cpp.i

CMakeFiles/radar.dir/dataprocessing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/radar.dir/dataprocessing.cpp.s"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/dataprocessing.cpp -o CMakeFiles/radar.dir/dataprocessing.cpp.s

CMakeFiles/radar.dir/dataprocessing.cpp.o.requires:

.PHONY : CMakeFiles/radar.dir/dataprocessing.cpp.o.requires

CMakeFiles/radar.dir/dataprocessing.cpp.o.provides: CMakeFiles/radar.dir/dataprocessing.cpp.o.requires
	$(MAKE) -f CMakeFiles/radar.dir/build.make CMakeFiles/radar.dir/dataprocessing.cpp.o.provides.build
.PHONY : CMakeFiles/radar.dir/dataprocessing.cpp.o.provides

CMakeFiles/radar.dir/dataprocessing.cpp.o.provides.build: CMakeFiles/radar.dir/dataprocessing.cpp.o


# Object files for target radar
radar_OBJECTS = \
"CMakeFiles/radar.dir/main.cpp.o" \
"CMakeFiles/radar.dir/radar.cpp.o" \
"CMakeFiles/radar.dir/dataprocessing.cpp.o"

# External object files for target radar
radar_EXTERNAL_OBJECTS =

radar: CMakeFiles/radar.dir/main.cpp.o
radar: CMakeFiles/radar.dir/radar.cpp.o
radar: CMakeFiles/radar.dir/dataprocessing.cpp.o
radar: CMakeFiles/radar.dir/build.make
radar: CMakeFiles/radar.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable radar"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/radar.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/radar.dir/build: radar

.PHONY : CMakeFiles/radar.dir/build

CMakeFiles/radar.dir/requires: CMakeFiles/radar.dir/main.cpp.o.requires
CMakeFiles/radar.dir/requires: CMakeFiles/radar.dir/radar.cpp.o.requires
CMakeFiles/radar.dir/requires: CMakeFiles/radar.dir/dataprocessing.cpp.o.requires

.PHONY : CMakeFiles/radar.dir/requires

CMakeFiles/radar.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/radar.dir/cmake_clean.cmake
.PHONY : CMakeFiles/radar.dir/clean

CMakeFiles/radar.dir/depend:
	cd /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02 /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02 /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/build /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/build /home/esteban/git/pfms-2020a-esteban-andrade/scratch/week07/starter/ex02/build/CMakeFiles/radar.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/radar.dir/depend

