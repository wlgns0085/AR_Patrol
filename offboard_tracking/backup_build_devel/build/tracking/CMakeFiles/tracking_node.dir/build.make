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
include tracking/CMakeFiles/tracking_node.dir/depend.make

# Include the progress variables for this target.
include tracking/CMakeFiles/tracking_node.dir/progress.make

# Include the compile flags for this target's objects.
include tracking/CMakeFiles/tracking_node.dir/flags.make

tracking/CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.o: tracking/CMakeFiles/tracking_node.dir/flags.make
tracking/CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.o: /home/nvidia/offboard_tracking/src/tracking/src/Tracking/ObjectTracking_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/offboard_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tracking/CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.o"
	cd /home/nvidia/offboard_tracking/build/tracking && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.o -c /home/nvidia/offboard_tracking/src/tracking/src/Tracking/ObjectTracking_node.cpp

tracking/CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.i"
	cd /home/nvidia/offboard_tracking/build/tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/offboard_tracking/src/tracking/src/Tracking/ObjectTracking_node.cpp > CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.i

tracking/CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.s"
	cd /home/nvidia/offboard_tracking/build/tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/offboard_tracking/src/tracking/src/Tracking/ObjectTracking_node.cpp -o CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.s

tracking/CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.o.requires:

.PHONY : tracking/CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.o.requires

tracking/CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.o.provides: tracking/CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.o.requires
	$(MAKE) -f tracking/CMakeFiles/tracking_node.dir/build.make tracking/CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.o.provides.build
.PHONY : tracking/CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.o.provides

tracking/CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.o.provides.build: tracking/CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.o


# Object files for target tracking_node
tracking_node_OBJECTS = \
"CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.o"

# External object files for target tracking_node
tracking_node_EXTERNAL_OBJECTS =

/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: tracking/CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.o
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: tracking/CMakeFiles/tracking_node.dir/build.make
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /home/nvidia/offboard_tracking/devel/lib/libtracking.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libmavros.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libGeographic.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libdiagnostic_updater.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libeigen_conversions.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libmavconn.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /home/nvidia/yolov3_ros/devel/lib/libdarknet_ros_lib.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libcv_bridge.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libimage_transport.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libclass_loader.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/libPocoFoundation.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libdl.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libroslib.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/librospack.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libimage_geometry.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_core.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_shape.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_superres.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_video.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_videostab.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_viz.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_aruco.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_bgsegm.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_bioinspired.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_ccalib.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_datasets.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_dpm.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_face.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_freetype.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_fuzzy.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_hdf.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_line_descriptor.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_optflow.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_plot.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_reg.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_rgbd.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_saliency.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_stereo.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_structured_light.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_surface_matching.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_text.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_ximgproc.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_xobjdetect.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libopencv_xphoto.so.3.2.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libcamera_info_manager.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libcamera_calibration_parsers.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libtf.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libtf2_ros.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libactionlib.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libmessage_filters.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libroscpp.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libtf2.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/librosconsole.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/librostime.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/libcpp_common.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: /opt/ros/melodic/lib/librealsense2.so.2.50.0
/home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node: tracking/CMakeFiles/tracking_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nvidia/offboard_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node"
	cd /home/nvidia/offboard_tracking/build/tracking && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tracking_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tracking/CMakeFiles/tracking_node.dir/build: /home/nvidia/offboard_tracking/devel/lib/tracking/tracking_node

.PHONY : tracking/CMakeFiles/tracking_node.dir/build

tracking/CMakeFiles/tracking_node.dir/requires: tracking/CMakeFiles/tracking_node.dir/src/Tracking/ObjectTracking_node.cpp.o.requires

.PHONY : tracking/CMakeFiles/tracking_node.dir/requires

tracking/CMakeFiles/tracking_node.dir/clean:
	cd /home/nvidia/offboard_tracking/build/tracking && $(CMAKE_COMMAND) -P CMakeFiles/tracking_node.dir/cmake_clean.cmake
.PHONY : tracking/CMakeFiles/tracking_node.dir/clean

tracking/CMakeFiles/tracking_node.dir/depend:
	cd /home/nvidia/offboard_tracking/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nvidia/offboard_tracking/src /home/nvidia/offboard_tracking/src/tracking /home/nvidia/offboard_tracking/build /home/nvidia/offboard_tracking/build/tracking /home/nvidia/offboard_tracking/build/tracking/CMakeFiles/tracking_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tracking/CMakeFiles/tracking_node.dir/depend
