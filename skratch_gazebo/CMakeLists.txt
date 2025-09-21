cmake_minimum_required(VERSION 3.5.1)
project(skratch_gazebo)

if(POLICY CMP0148) 
	cmake_policy(SET CMP0148 OLD) 
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(kelo_tulip REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_ign_bridge REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# The documentation build option set to OFF by default
option(BUILD_DOC "Build documentation" OFF)
if(BUILD_DOC)
    # check if Doxygen is installed
    find_package(Doxygen)
    if (DOXYGEN_FOUND)
        # set input and output files
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/config/doxygen/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        message("Doxygen build started")
        # note the option ALL which allows to build the docs together with the application
        add_custom_target( doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
    else (DOXYGEN_FOUND)
        message("Doxygen need to be installed to generate the doxygen documentation")
    endif (DOXYGEN_FOUND)
endif(BUILD_DOC)

include_directories(
    include
    ${kelo_tulip_INCLUDE_DIRS})

add_executable(skratch_gazebo_platform_controller
    src/KeloDrive.cpp
    src/SkratchPlatformController.cpp
    src/main.cpp
)

ament_target_dependencies(skratch_gazebo_platform_controller
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  rviz_common
  rviz_ogre_vendor
  rviz_rendering
  ros_gz_sim
  ros_gz_bridge   
  kelo_tulip
)

# Install targets and directories
install(TARGETS
  skratch_gazebo_platform_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)
# ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(skratch_gazebo_platform_controller)
ament_package()

