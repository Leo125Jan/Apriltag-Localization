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

# Utility rule file for testplot_autogen.

# Include the progress variables for this target.
include final_project/multirotor_geometry_control/plottest/CMakeFiles/testplot_autogen.dir/progress.make

final_project/multirotor_geometry_control/plottest/CMakeFiles/testplot_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/leo/UAY/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target testplot"
	cd /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest && /usr/bin/cmake -E cmake_autogen /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest/CMakeFiles/testplot_autogen.dir/AutogenInfo.json Release

testplot_autogen: final_project/multirotor_geometry_control/plottest/CMakeFiles/testplot_autogen
testplot_autogen: final_project/multirotor_geometry_control/plottest/CMakeFiles/testplot_autogen.dir/build.make

.PHONY : testplot_autogen

# Rule to build all files generated by this target.
final_project/multirotor_geometry_control/plottest/CMakeFiles/testplot_autogen.dir/build: testplot_autogen

.PHONY : final_project/multirotor_geometry_control/plottest/CMakeFiles/testplot_autogen.dir/build

final_project/multirotor_geometry_control/plottest/CMakeFiles/testplot_autogen.dir/clean:
	cd /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest && $(CMAKE_COMMAND) -P CMakeFiles/testplot_autogen.dir/cmake_clean.cmake
.PHONY : final_project/multirotor_geometry_control/plottest/CMakeFiles/testplot_autogen.dir/clean

final_project/multirotor_geometry_control/plottest/CMakeFiles/testplot_autogen.dir/depend:
	cd /home/leo/UAY/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leo/UAY/src /home/leo/UAY/src/final_project/multirotor_geometry_control/plottest /home/leo/UAY/build /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest /home/leo/UAY/build/final_project/multirotor_geometry_control/plottest/CMakeFiles/testplot_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : final_project/multirotor_geometry_control/plottest/CMakeFiles/testplot_autogen.dir/depend

