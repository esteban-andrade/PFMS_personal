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
CMAKE_SOURCE_DIR = /home/esteban/git/pfms-2020a-esteban-andrade/tutorials/week08/starter/opencv/access_pixels

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/esteban/git/pfms-2020a-esteban-andrade/tutorials/week08/starter/opencv/access_pixels/build

# Include any dependencies generated for this target.
include CMakeFiles/access_pixels.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/access_pixels.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/access_pixels.dir/flags.make

CMakeFiles/access_pixels.dir/access_pixels.cpp.o: CMakeFiles/access_pixels.dir/flags.make
CMakeFiles/access_pixels.dir/access_pixels.cpp.o: ../access_pixels.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/esteban/git/pfms-2020a-esteban-andrade/tutorials/week08/starter/opencv/access_pixels/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/access_pixels.dir/access_pixels.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/access_pixels.dir/access_pixels.cpp.o -c /home/esteban/git/pfms-2020a-esteban-andrade/tutorials/week08/starter/opencv/access_pixels/access_pixels.cpp

CMakeFiles/access_pixels.dir/access_pixels.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/access_pixels.dir/access_pixels.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/esteban/git/pfms-2020a-esteban-andrade/tutorials/week08/starter/opencv/access_pixels/access_pixels.cpp > CMakeFiles/access_pixels.dir/access_pixels.cpp.i

CMakeFiles/access_pixels.dir/access_pixels.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/access_pixels.dir/access_pixels.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/esteban/git/pfms-2020a-esteban-andrade/tutorials/week08/starter/opencv/access_pixels/access_pixels.cpp -o CMakeFiles/access_pixels.dir/access_pixels.cpp.s

CMakeFiles/access_pixels.dir/access_pixels.cpp.o.requires:

.PHONY : CMakeFiles/access_pixels.dir/access_pixels.cpp.o.requires

CMakeFiles/access_pixels.dir/access_pixels.cpp.o.provides: CMakeFiles/access_pixels.dir/access_pixels.cpp.o.requires
	$(MAKE) -f CMakeFiles/access_pixels.dir/build.make CMakeFiles/access_pixels.dir/access_pixels.cpp.o.provides.build
.PHONY : CMakeFiles/access_pixels.dir/access_pixels.cpp.o.provides

CMakeFiles/access_pixels.dir/access_pixels.cpp.o.provides.build: CMakeFiles/access_pixels.dir/access_pixels.cpp.o


# Object files for target access_pixels
access_pixels_OBJECTS = \
"CMakeFiles/access_pixels.dir/access_pixels.cpp.o"

# External object files for target access_pixels
access_pixels_EXTERNAL_OBJECTS =

access_pixels: CMakeFiles/access_pixels.dir/access_pixels.cpp.o
access_pixels: CMakeFiles/access_pixels.dir/build.make
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
access_pixels: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
access_pixels: CMakeFiles/access_pixels.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/esteban/git/pfms-2020a-esteban-andrade/tutorials/week08/starter/opencv/access_pixels/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable access_pixels"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/access_pixels.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/access_pixels.dir/build: access_pixels

.PHONY : CMakeFiles/access_pixels.dir/build

CMakeFiles/access_pixels.dir/requires: CMakeFiles/access_pixels.dir/access_pixels.cpp.o.requires

.PHONY : CMakeFiles/access_pixels.dir/requires

CMakeFiles/access_pixels.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/access_pixels.dir/cmake_clean.cmake
.PHONY : CMakeFiles/access_pixels.dir/clean

CMakeFiles/access_pixels.dir/depend:
	cd /home/esteban/git/pfms-2020a-esteban-andrade/tutorials/week08/starter/opencv/access_pixels/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/esteban/git/pfms-2020a-esteban-andrade/tutorials/week08/starter/opencv/access_pixels /home/esteban/git/pfms-2020a-esteban-andrade/tutorials/week08/starter/opencv/access_pixels /home/esteban/git/pfms-2020a-esteban-andrade/tutorials/week08/starter/opencv/access_pixels/build /home/esteban/git/pfms-2020a-esteban-andrade/tutorials/week08/starter/opencv/access_pixels/build /home/esteban/git/pfms-2020a-esteban-andrade/tutorials/week08/starter/opencv/access_pixels/build/CMakeFiles/access_pixels.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/access_pixels.dir/depend

