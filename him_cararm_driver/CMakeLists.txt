cmake_minimum_required(VERSION 2.8.3)
project(him_cararm_driver)
add_definitions( -DROS_BUILD )


find_package(catkin REQUIRED COMPONENTS

  actionlib
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  actionlib_msgs
  tf
  serial
)


## Generate actions in the 'action' folder
 add_action_files(
   FILES
   WayPointPose.action
   CarPose.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
 )


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES arm_driver_v2
  CATKIN_DEPENDS  actionlib geometry_msgs roscpp sensor_msgs actionlib_msgs
)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()


include_directories(include 
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
set(${PROJECT_NAME}_SOURCES
    src/arm_ros_wrapper.cpp
    src/Communicate.cpp
    src/motion_planning.cpp
    src/fk_ik.cpp
)
add_executable(him_cararm_driver ${${PROJECT_NAME}_SOURCES})

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(him_cararm_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(him_cararm_driver
  ${catkin_LIBRARIES}
 )

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

## Mark executables and/or libraries for installation
install(TARGETS him_cararm_driver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

