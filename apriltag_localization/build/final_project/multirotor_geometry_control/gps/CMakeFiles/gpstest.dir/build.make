# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/leo/UAY/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leo/UAY/build

# Include any dependencies generated for this target.
include final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/depend.make

# Include the progress variables for this target.
include final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/progress.make

# Include the compile flags for this target's objects.
include final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/flags.make

final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/src/gpstest.cpp.o: final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/flags.make
final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/src/gpstest.cpp.o: /home/leo/UAY/src/final_project/multirotor_geometry_control/gps/src/gpstest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UAY/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/src/gpstest.cpp.o"
	cd /home/leo/UAY/build/final_project/multirotor_geometry_control/gps && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gpstest.dir/src/gpstest.cpp.o -c /home/leo/UAY/src/final_project/multirotor_geometry_control/gps/src/gpstest.cpp

final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/src/gpstest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gpstest.dir/src/gpstest.cpp.i"
	cd /home/leo/UAY/build/final_project/multirotor_geometry_control/gps && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UAY/src/final_project/multirotor_geometry_control/gps/src/gpstest.cpp > CMakeFiles/gpstest.dir/src/gpstest.cpp.i

final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/src/gpstest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gpstest.dir/src/gpstest.cpp.s"
	cd /home/leo/UAY/build/final_project/multirotor_geometry_control/gps && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UAY/src/final_project/multirotor_geometry_control/gps/src/gpstest.cpp -o CMakeFiles/gpstest.dir/src/gpstest.cpp.s

# Object files for target gpstest
gpstest_OBJECTS = \
"CMakeFiles/gpstest.dir/src/gpstest.cpp.o"

# External object files for target gpstest
gpstest_EXTERNAL_OBJECTS =

/home/leo/UAY/devel/lib/gps/gpstest: final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/src/gpstest.cpp.o
/home/leo/UAY/devel/lib/gps/gpstest: final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/build.make
/home/leo/UAY/devel/lib/gps/gpstest: /opt/ros/noetic/lib/libroscpp.so
/home/leo/UAY/devel/lib/gps/gpstest: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/leo/UAY/devel/lib/gps/gpstest: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/leo/UAY/devel/lib/gps/gpstest: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/leo/UAY/devel/lib/gps/gpstest: /opt/ros/noetic/lib/librosconsole.so
/home/leo/UAY/devel/lib/gps/gpstest: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/leo/UAY/devel/lib/gps/gpstest: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/leo/UAY/devel/lib/gps/gpstest: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/leo/UAY/devel/lib/gps/gpstest: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/leo/UAY/devel/lib/gps/gpstest: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/leo/UAY/devel/lib/gps/gpstest: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/leo/UAY/devel/lib/gps/gpstest: /opt/ros/noetic/lib/librostime.so
/home/leo/UAY/devel/lib/gps/gpstest: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/leo/UAY/devel/lib/gps/gpstest: /opt/ros/noetic/lib/libcpp_common.so
/home/leo/UAY/devel/lib/gps/gpstest: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/leo/UAY/devel/lib/gps/gpstest: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/leo/UAY/devel/lib/gps/gpstest: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/leo/UAY/devel/lib/gps/gpstest: /usr/lib/x86_64-linux-gnu/libGeographic.so
/home/leo/UAY/devel/lib/gps/gpstest: final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leo/UAY/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/leo/UAY/devel/lib/gps/gpstest"
	cd /home/leo/UAY/build/final_project/multirotor_geometry_control/gps && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gpstest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/build: /home/leo/UAY/devel/lib/gps/gpstest

.PHONY : final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/build

final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/clean:
	cd /home/leo/UAY/build/final_project/multirotor_geometry_control/gps && $(CMAKE_COMMAND) -P CMakeFiles/gpstest.dir/cmake_clean.cmake
.PHONY : final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/clean

final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/depend:
	cd /home/leo/UAY/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leo/UAY/src /home/leo/UAY/src/final_project/multirotor_geometry_control/gps /home/leo/UAY/build /home/leo/UAY/build/final_project/multirotor_geometry_control/gps /home/leo/UAY/build/final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : final_project/multirotor_geometry_control/gps/CMakeFiles/gpstest.dir/depend

