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
CMAKE_SOURCE_DIR = /home/leo/UAV/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leo/UAV/build

# Include any dependencies generated for this target.
include final_project/offb/CMakeFiles/geo2.dir/depend.make

# Include the progress variables for this target.
include final_project/offb/CMakeFiles/geo2.dir/progress.make

# Include the compile flags for this target's objects.
include final_project/offb/CMakeFiles/geo2.dir/flags.make

final_project/offb/CMakeFiles/geo2.dir/src/geo2.cpp.o: final_project/offb/CMakeFiles/geo2.dir/flags.make
final_project/offb/CMakeFiles/geo2.dir/src/geo2.cpp.o: /home/leo/UAV/src/final_project/offb/src/geo2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UAV/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object final_project/offb/CMakeFiles/geo2.dir/src/geo2.cpp.o"
	cd /home/leo/UAV/build/final_project/offb && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/geo2.dir/src/geo2.cpp.o -c /home/leo/UAV/src/final_project/offb/src/geo2.cpp

final_project/offb/CMakeFiles/geo2.dir/src/geo2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/geo2.dir/src/geo2.cpp.i"
	cd /home/leo/UAV/build/final_project/offb && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UAV/src/final_project/offb/src/geo2.cpp > CMakeFiles/geo2.dir/src/geo2.cpp.i

final_project/offb/CMakeFiles/geo2.dir/src/geo2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/geo2.dir/src/geo2.cpp.s"
	cd /home/leo/UAV/build/final_project/offb && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UAV/src/final_project/offb/src/geo2.cpp -o CMakeFiles/geo2.dir/src/geo2.cpp.s

final_project/offb/CMakeFiles/geo2.dir/src/geo2.cpp.o.requires:

.PHONY : final_project/offb/CMakeFiles/geo2.dir/src/geo2.cpp.o.requires

final_project/offb/CMakeFiles/geo2.dir/src/geo2.cpp.o.provides: final_project/offb/CMakeFiles/geo2.dir/src/geo2.cpp.o.requires
	$(MAKE) -f final_project/offb/CMakeFiles/geo2.dir/build.make final_project/offb/CMakeFiles/geo2.dir/src/geo2.cpp.o.provides.build
.PHONY : final_project/offb/CMakeFiles/geo2.dir/src/geo2.cpp.o.provides

final_project/offb/CMakeFiles/geo2.dir/src/geo2.cpp.o.provides.build: final_project/offb/CMakeFiles/geo2.dir/src/geo2.cpp.o


# Object files for target geo2
geo2_OBJECTS = \
"CMakeFiles/geo2.dir/src/geo2.cpp.o"

# External object files for target geo2
geo2_EXTERNAL_OBJECTS =

/home/leo/UAV/devel/lib/offb/geo2: final_project/offb/CMakeFiles/geo2.dir/src/geo2.cpp.o
/home/leo/UAV/devel/lib/offb/geo2: final_project/offb/CMakeFiles/geo2.dir/build.make
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libgazebo_ros_api_plugin.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libgazebo_ros_paths_plugin.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libtf.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libmavros.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libGeographic.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libdiagnostic_updater.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libeigen_conversions.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libmavconn.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libclass_loader.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/libPocoFoundation.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libdl.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libroslib.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/librospack.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libtf2_ros.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libactionlib.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libmessage_filters.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libroscpp.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/librosconsole.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libtf2.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/librostime.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libcpp_common.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/leo/UAV/devel/lib/offb/geo2: /home/leo/UAV/devel/lib/libqptrajectory.so
/home/leo/UAV/devel/lib/offb/geo2: /home/leo/UAV/devel/lib/libgeometric_controller.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libgazebo_ros_api_plugin.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libgazebo_ros_paths_plugin.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libtf.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libmavros.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libGeographic.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libdiagnostic_updater.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libeigen_conversions.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libmavconn.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libclass_loader.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/libPocoFoundation.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libdl.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libroslib.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/librospack.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libtf2_ros.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libactionlib.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libmessage_filters.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libroscpp.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/librosconsole.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libtf2.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/librostime.so
/home/leo/UAV/devel/lib/offb/geo2: /opt/ros/melodic/lib/libcpp_common.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/leo/UAV/devel/lib/offb/geo2: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/leo/UAV/devel/lib/offb/geo2: final_project/offb/CMakeFiles/geo2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leo/UAV/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/leo/UAV/devel/lib/offb/geo2"
	cd /home/leo/UAV/build/final_project/offb && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/geo2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
final_project/offb/CMakeFiles/geo2.dir/build: /home/leo/UAV/devel/lib/offb/geo2

.PHONY : final_project/offb/CMakeFiles/geo2.dir/build

final_project/offb/CMakeFiles/geo2.dir/requires: final_project/offb/CMakeFiles/geo2.dir/src/geo2.cpp.o.requires

.PHONY : final_project/offb/CMakeFiles/geo2.dir/requires

final_project/offb/CMakeFiles/geo2.dir/clean:
	cd /home/leo/UAV/build/final_project/offb && $(CMAKE_COMMAND) -P CMakeFiles/geo2.dir/cmake_clean.cmake
.PHONY : final_project/offb/CMakeFiles/geo2.dir/clean

final_project/offb/CMakeFiles/geo2.dir/depend:
	cd /home/leo/UAV/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leo/UAV/src /home/leo/UAV/src/final_project/offb /home/leo/UAV/build /home/leo/UAV/build/final_project/offb /home/leo/UAV/build/final_project/offb/CMakeFiles/geo2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : final_project/offb/CMakeFiles/geo2.dir/depend

