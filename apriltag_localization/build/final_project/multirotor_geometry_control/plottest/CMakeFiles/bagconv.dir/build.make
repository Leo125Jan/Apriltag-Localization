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
include final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/depend.make

# Include the progress variables for this target.
include final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/progress.make

# Include the compile flags for this target's objects.
include final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/flags.make

final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/bagconv_autogen/mocs_compilation.cpp.o: final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/flags.make
final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/bagconv_autogen/mocs_compilation.cpp.o: final_project/multirotor_geometry_control/plottest/bagconv_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UAY/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/bagconv_autogen/mocs_compilation.cpp.o"
	cd /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bagconv.dir/bagconv_autogen/mocs_compilation.cpp.o -c /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest/bagconv_autogen/mocs_compilation.cpp

final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/bagconv_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bagconv.dir/bagconv_autogen/mocs_compilation.cpp.i"
	cd /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest/bagconv_autogen/mocs_compilation.cpp > CMakeFiles/bagconv.dir/bagconv_autogen/mocs_compilation.cpp.i

final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/bagconv_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bagconv.dir/bagconv_autogen/mocs_compilation.cpp.s"
	cd /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest/bagconv_autogen/mocs_compilation.cpp -o CMakeFiles/bagconv.dir/bagconv_autogen/mocs_compilation.cpp.s

final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/src/bagconv.cpp.o: final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/flags.make
final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/src/bagconv.cpp.o: /home/leo/UAY/src/final_project/multirotor_geometry_control/plottest/src/bagconv.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/UAY/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/src/bagconv.cpp.o"
	cd /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bagconv.dir/src/bagconv.cpp.o -c /home/leo/UAY/src/final_project/multirotor_geometry_control/plottest/src/bagconv.cpp

final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/src/bagconv.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bagconv.dir/src/bagconv.cpp.i"
	cd /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/UAY/src/final_project/multirotor_geometry_control/plottest/src/bagconv.cpp > CMakeFiles/bagconv.dir/src/bagconv.cpp.i

final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/src/bagconv.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bagconv.dir/src/bagconv.cpp.s"
	cd /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/UAY/src/final_project/multirotor_geometry_control/plottest/src/bagconv.cpp -o CMakeFiles/bagconv.dir/src/bagconv.cpp.s

# Object files for target bagconv
bagconv_OBJECTS = \
"CMakeFiles/bagconv.dir/bagconv_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/bagconv.dir/src/bagconv.cpp.o"

# External object files for target bagconv
bagconv_EXTERNAL_OBJECTS =

/home/leo/UAY/devel/lib/plottest/bagconv: final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/bagconv_autogen/mocs_compilation.cpp.o
/home/leo/UAY/devel/lib/plottest/bagconv: final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/src/bagconv.cpp.o
/home/leo/UAY/devel/lib/plottest/bagconv: final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/build.make
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/libmavros.so
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/x86_64-linux-gnu/libGeographic.so
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/libdiagnostic_updater.so
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/libeigen_conversions.so
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/liborocos-kdl.so
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/libmavconn.so
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/libclass_loader.so
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/x86_64-linux-gnu/libPocoFoundation.so
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/x86_64-linux-gnu/libdl.so
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/libroslib.so
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/librospack.so
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/x86_64-linux-gnu/libpython3.8.so
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/libtf2_ros.so
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/libactionlib.so
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/libmessage_filters.so
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/libroscpp.so
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/librosconsole.so
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/libtf2.so
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/librostime.so
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/leo/UAY/devel/lib/plottest/bagconv: /opt/ros/noetic/lib/libcpp_common.so
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/leo/UAY/devel/lib/plottest/bagconv: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/leo/UAY/devel/lib/plottest/bagconv: final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leo/UAY/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/leo/UAY/devel/lib/plottest/bagconv"
	cd /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bagconv.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/build: /home/leo/UAY/devel/lib/plottest/bagconv

.PHONY : final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/build

final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/clean:
	cd /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest && $(CMAKE_COMMAND) -P CMakeFiles/bagconv.dir/cmake_clean.cmake
.PHONY : final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/clean

final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/depend:
	cd /home/leo/UAY/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leo/UAY/src /home/leo/UAY/src/final_project/multirotor_geometry_control/plottest /home/leo/UAY/build /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : final_project/multirotor_geometry_control/plottest/CMakeFiles/bagconv.dir/depend

