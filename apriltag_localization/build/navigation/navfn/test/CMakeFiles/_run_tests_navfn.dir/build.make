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

# Utility rule file for _run_tests_navfn.

# Include the progress variables for this target.
include navigation/navfn/test/CMakeFiles/_run_tests_navfn.dir/progress.make

_run_tests_navfn: navigation/navfn/test/CMakeFiles/_run_tests_navfn.dir/build.make

.PHONY : _run_tests_navfn

# Rule to build all files generated by this target.
navigation/navfn/test/CMakeFiles/_run_tests_navfn.dir/build: _run_tests_navfn

.PHONY : navigation/navfn/test/CMakeFiles/_run_tests_navfn.dir/build

navigation/navfn/test/CMakeFiles/_run_tests_navfn.dir/clean:
	cd /home/leo/UAV/build/navigation/navfn/test && $(CMAKE_COMMAND) -P CMakeFiles/_run_tests_navfn.dir/cmake_clean.cmake
.PHONY : navigation/navfn/test/CMakeFiles/_run_tests_navfn.dir/clean

navigation/navfn/test/CMakeFiles/_run_tests_navfn.dir/depend:
	cd /home/leo/UAV/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leo/UAV/src /home/leo/UAV/src/navigation/navfn/test /home/leo/UAV/build /home/leo/UAV/build/navigation/navfn/test /home/leo/UAV/build/navigation/navfn/test/CMakeFiles/_run_tests_navfn.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : navigation/navfn/test/CMakeFiles/_run_tests_navfn.dir/depend

