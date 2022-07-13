# Install script for directory: /home/leo/UAY/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/leo/UAY/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/leo/UAY/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/leo/UAY/install" TYPE PROGRAM FILES "/home/leo/UAY/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/leo/UAY/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/leo/UAY/install" TYPE PROGRAM FILES "/home/leo/UAY/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/leo/UAY/install/setup.bash;/home/leo/UAY/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/leo/UAY/install" TYPE FILE FILES
    "/home/leo/UAY/build/catkin_generated/installspace/setup.bash"
    "/home/leo/UAY/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/leo/UAY/install/setup.sh;/home/leo/UAY/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/leo/UAY/install" TYPE FILE FILES
    "/home/leo/UAY/build/catkin_generated/installspace/setup.sh"
    "/home/leo/UAY/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/leo/UAY/install/setup.zsh;/home/leo/UAY/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/leo/UAY/install" TYPE FILE FILES
    "/home/leo/UAY/build/catkin_generated/installspace/setup.zsh"
    "/home/leo/UAY/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/leo/UAY/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/leo/UAY/install" TYPE FILE FILES "/home/leo/UAY/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/leo/UAY/build/gtest/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/husky/husky_desktop/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/husky/husky_simulator/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/mav_comm-3.3.2/mav_comm/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/navigation/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_description/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_simulator/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/husky/husky_msgs/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_comm/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/mav_comm-3.3.2/mav_msgs/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/multirotor_geometry_control/gps/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/multirotor_geometry_control/plottest/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/husky/husky_control/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/husky/husky_description/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/husky/husky_gazebo/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/husky/husky_navigation/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/husky/husky_viz/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/map_server/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_evaluation/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/mav_comm-3.3.2/mav_planning_msgs/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_control/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_joy_interface/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/multirotor_geometry_control/rotors_simulator/rqt_rotors/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_hil_interface/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/apriltag_ros/apriltag_ros/cmake_install.cmake")
  include("/home/leo/UAY/build/gazebo_ros_link_attacher/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/multirotor_geometry_control/path_planner/cmake_install.cmake")
  include("/home/leo/UAY/build/realsense_gazebo_plugin/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/amcl/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/fake_localization/cmake_install.cmake")
  include("/home/leo/UAY/build/robot_localization-noetic-devel/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo_plugins/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/voxel_grid/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/costmap_2d/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/nav_core/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/base_local_planner/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/carrot_planner/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/clear_costmap_recovery/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/dwa_local_planner/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/move_slow_and_clear/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/navfn/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/global_planner/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/rotate_recovery/cmake_install.cmake")
  include("/home/leo/UAY/build/navigation/move_base/cmake_install.cmake")
  include("/home/leo/UAY/build/final_project/multirotor_geometry_control/rotors_simulator/rotors_gazebo/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/leo/UAY/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
