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
include final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/depend.make

# Include the progress variables for this target.
include final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/progress.make

# Include the compile flags for this target's objects.
include final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/flags.make

final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o: final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/flags.make
final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o: /home/leo/UAV/src/final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/src/hovering_example.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UAV/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o"
	cd /home/leo/UAV/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o -c /home/leo/UAV/src/final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/src/hovering_example.cpp

final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/src/hovering_example.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hovering_example.dir/src/hovering_example.cpp.i"
	cd /home/leo/UAV/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UAV/src/final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/src/hovering_example.cpp > CMakeFiles/hovering_example.dir/src/hovering_example.cpp.i

final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/src/hovering_example.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hovering_example.dir/src/hovering_example.cpp.s"
	cd /home/leo/UAV/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UAV/src/final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/src/hovering_example.cpp -o CMakeFiles/hovering_example.dir/src/hovering_example.cpp.s

final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o.requires:

.PHONY : final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o.requires

final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o.provides: final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o.requires
	$(MAKE) -f final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/build.make final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o.provides.build
.PHONY : final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o.provides

final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o.provides.build: final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o


# Object files for target hovering_example
hovering_example_OBJECTS = \
"CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o"

# External object files for target hovering_example
hovering_example_EXTERNAL_OBJECTS =

/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/build.make
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /opt/ros/melodic/lib/libroscpp.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /opt/ros/melodic/lib/librosconsole.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /opt/ros/melodic/lib/librostime.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /opt/ros/melodic/lib/libcpp_common.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/leo/UAV/devel/lib/rotors_gazebo/hovering_example: final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leo/UAV/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/leo/UAV/devel/lib/rotors_gazebo/hovering_example"
	cd /home/leo/UAV/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hovering_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/build: /home/leo/UAV/devel/lib/rotors_gazebo/hovering_example

.PHONY : final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/build

final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/requires: final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o.requires

.PHONY : final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/requires

final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/clean:
	cd /home/leo/UAV/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo && $(CMAKE_COMMAND) -P CMakeFiles/hovering_example.dir/cmake_clean.cmake
.PHONY : final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/clean

final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/depend:
	cd /home/leo/UAV/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leo/UAV/src /home/leo/UAV/src/final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo /home/leo/UAV/build /home/leo/UAV/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo /home/leo/UAV/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/CMakeFiles/hovering_example.dir/depend

