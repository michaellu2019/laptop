# Install script for directory: /home/mlu/Documents/Programming/ROS/laptop_ws/src/arm_link_assembly_urdf

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/mlu/Documents/Programming/ROS/laptop_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/mlu/Documents/Programming/ROS/laptop_ws/build/arm_link_assembly_urdf/catkin_generated/installspace/arm_link_assembly_urdf.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_link_assembly_urdf/cmake" TYPE FILE FILES
    "/home/mlu/Documents/Programming/ROS/laptop_ws/build/arm_link_assembly_urdf/catkin_generated/installspace/arm_link_assembly_urdfConfig.cmake"
    "/home/mlu/Documents/Programming/ROS/laptop_ws/build/arm_link_assembly_urdf/catkin_generated/installspace/arm_link_assembly_urdfConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_link_assembly_urdf" TYPE FILE FILES "/home/mlu/Documents/Programming/ROS/laptop_ws/src/arm_link_assembly_urdf/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_link_assembly_urdf/config" TYPE DIRECTORY FILES "/home/mlu/Documents/Programming/ROS/laptop_ws/src/arm_link_assembly_urdf/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_link_assembly_urdf/launch" TYPE DIRECTORY FILES "/home/mlu/Documents/Programming/ROS/laptop_ws/src/arm_link_assembly_urdf/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_link_assembly_urdf/meshes" TYPE DIRECTORY FILES "/home/mlu/Documents/Programming/ROS/laptop_ws/src/arm_link_assembly_urdf/meshes/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_link_assembly_urdf/urdf" TYPE DIRECTORY FILES "/home/mlu/Documents/Programming/ROS/laptop_ws/src/arm_link_assembly_urdf/urdf/")
endif()

