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
CMAKE_SOURCE_DIR = /home/nvidia/offboard_tracking/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nvidia/offboard_tracking/build

# Include any dependencies generated for this target.
include tracking/CMakeFiles/tracking.dir/depend.make

# Include the progress variables for this target.
include tracking/CMakeFiles/tracking.dir/progress.make

# Include the compile flags for this target's objects.
include tracking/CMakeFiles/tracking.dir/flags.make

tracking/CMakeFiles/tracking.dir/src/Common/Convert.cpp.o: tracking/CMakeFiles/tracking.dir/flags.make
tracking/CMakeFiles/tracking.dir/src/Common/Convert.cpp.o: /home/nvidia/offboard_tracking/src/tracking/src/Common/Convert.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/offboard_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tracking/CMakeFiles/tracking.dir/src/Common/Convert.cpp.o"
	cd /home/nvidia/offboard_tracking/build/tracking && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/src/Common/Convert.cpp.o -c /home/nvidia/offboard_tracking/src/tracking/src/Common/Convert.cpp

tracking/CMakeFiles/tracking.dir/src/Common/Convert.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/src/Common/Convert.cpp.i"
	cd /home/nvidia/offboard_tracking/build/tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/offboard_tracking/src/tracking/src/Common/Convert.cpp > CMakeFiles/tracking.dir/src/Common/Convert.cpp.i

tracking/CMakeFiles/tracking.dir/src/Common/Convert.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/src/Common/Convert.cpp.s"
	cd /home/nvidia/offboard_tracking/build/tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/offboard_tracking/src/tracking/src/Common/Convert.cpp -o CMakeFiles/tracking.dir/src/Common/Convert.cpp.s

tracking/CMakeFiles/tracking.dir/src/Common/Convert.cpp.o.requires:

.PHONY : tracking/CMakeFiles/tracking.dir/src/Common/Convert.cpp.o.requires

tracking/CMakeFiles/tracking.dir/src/Common/Convert.cpp.o.provides: tracking/CMakeFiles/tracking.dir/src/Common/Convert.cpp.o.requires
	$(MAKE) -f tracking/CMakeFiles/tracking.dir/build.make tracking/CMakeFiles/tracking.dir/src/Common/Convert.cpp.o.provides.build
.PHONY : tracking/CMakeFiles/tracking.dir/src/Common/Convert.cpp.o.provides

tracking/CMakeFiles/tracking.dir/src/Common/Convert.cpp.o.provides.build: tracking/CMakeFiles/tracking.dir/src/Common/Convert.cpp.o


tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.o: tracking/CMakeFiles/tracking.dir/flags.make
tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.o: /home/nvidia/offboard_tracking/src/tracking/src/Tracking/ObjectTracking.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/offboard_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.o"
	cd /home/nvidia/offboard_tracking/build/tracking && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.o -c /home/nvidia/offboard_tracking/src/tracking/src/Tracking/ObjectTracking.cpp

tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.i"
	cd /home/nvidia/offboard_tracking/build/tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/offboard_tracking/src/tracking/src/Tracking/ObjectTracking.cpp > CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.i

tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.s"
	cd /home/nvidia/offboard_tracking/build/tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/offboard_tracking/src/tracking/src/Tracking/ObjectTracking.cpp -o CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.s

tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.o.requires:

.PHONY : tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.o.requires

tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.o.provides: tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.o.requires
	$(MAKE) -f tracking/CMakeFiles/tracking.dir/build.make tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.o.provides.build
.PHONY : tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.o.provides

tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.o.provides.build: tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.o


tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.o: tracking/CMakeFiles/tracking.dir/flags.make
tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.o: /home/nvidia/offboard_tracking/src/tracking/src/Tracking/ObjectTracking_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/offboard_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.o"
	cd /home/nvidia/offboard_tracking/build/tracking && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.o -c /home/nvidia/offboard_tracking/src/tracking/src/Tracking/ObjectTracking_node.cpp

tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.i"
	cd /home/nvidia/offboard_tracking/build/tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/offboard_tracking/src/tracking/src/Tracking/ObjectTracking_node.cpp > CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.i

tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.s"
	cd /home/nvidia/offboard_tracking/build/tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/offboard_tracking/src/tracking/src/Tracking/ObjectTracking_node.cpp -o CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.s

tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.o.requires:

.PHONY : tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.o.requires

tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.o.provides: tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.o.requires
	$(MAKE) -f tracking/CMakeFiles/tracking.dir/build.make tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.o.provides.build
.PHONY : tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.o.provides

tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.o.provides.build: tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.o


# Object files for target tracking
tracking_OBJECTS = \
"CMakeFiles/tracking.dir/src/Common/Convert.cpp.o" \
"CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.o" \
"CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.o"

# External object files for target tracking
tracking_EXTERNAL_OBJECTS =

/home/nvidia/offboard_tracking/devel/lib/libtracking.so: tracking/CMakeFiles/tracking.dir/src/Common/Convert.cpp.o
/home/nvidia/offboard_tracking/devel/lib/libtracking.so: tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.o
/home/nvidia/offboard_tracking/devel/lib/libtracking.so: tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.o
/home/nvidia/offboard_tracking/devel/lib/libtracking.so: tracking/CMakeFiles/tracking.dir/build.make
/home/nvidia/offboard_tracking/devel/lib/libtracking.so: tracking/CMakeFiles/tracking.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nvidia/offboard_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library /home/nvidia/offboard_tracking/devel/lib/libtracking.so"
	cd /home/nvidia/offboard_tracking/build/tracking && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tracking.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tracking/CMakeFiles/tracking.dir/build: /home/nvidia/offboard_tracking/devel/lib/libtracking.so

.PHONY : tracking/CMakeFiles/tracking.dir/build

tracking/CMakeFiles/tracking.dir/requires: tracking/CMakeFiles/tracking.dir/src/Common/Convert.cpp.o.requires
tracking/CMakeFiles/tracking.dir/requires: tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking.cpp.o.requires
tracking/CMakeFiles/tracking.dir/requires: tracking/CMakeFiles/tracking.dir/src/Tracking/ObjectTracking_node.cpp.o.requires

.PHONY : tracking/CMakeFiles/tracking.dir/requires

tracking/CMakeFiles/tracking.dir/clean:
	cd /home/nvidia/offboard_tracking/build/tracking && $(CMAKE_COMMAND) -P CMakeFiles/tracking.dir/cmake_clean.cmake
.PHONY : tracking/CMakeFiles/tracking.dir/clean

tracking/CMakeFiles/tracking.dir/depend:
	cd /home/nvidia/offboard_tracking/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nvidia/offboard_tracking/src /home/nvidia/offboard_tracking/src/tracking /home/nvidia/offboard_tracking/build /home/nvidia/offboard_tracking/build/tracking /home/nvidia/offboard_tracking/build/tracking/CMakeFiles/tracking.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tracking/CMakeFiles/tracking.dir/depend

