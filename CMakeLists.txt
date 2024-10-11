cmake_minimum_required(VERSION 3.5)
project(wolf_gazebo_interface)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 nor C++14 support. Please use a different C++ compiler.")
endif()

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)
find_package(pluginlib REQUIRED)

if(${catkin_FOUND})
    set(ROS_VERSION "ros")
    catkin_package(INCLUDE_DIRS include LIBRARIES wolf_hw_sim CATKIN_DEPENDS wolf_hardware_interface)
    add_definitions(-DROS)
    message(STATUS "Add ROS definition")
elseif(${ament_cmake_FOUND})
    set(ROS_VERSION "ros2")
    pluginlib_export_plugin_description_file(hardware_interface wolf_hw_sim_plugins.xml)
    ament_export_include_directories(include)
    ament_export_libraries(wolf_hw_sim)
    ament_package()
    add_definitions(-DROS2)
    message(STATUS "Add ROS2 definition")
endif()

find_package(gazebo_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(wolf_hardware_interface REQUIRED)
find_package(wolf_controller_utils REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(gazebo_${ROS_VERSION}_control REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  include/${ROS_VERSION}
  ${gazebo_ros_INCLUDE_DIRS}
  ${hardware_interface_INCLUDE_DIRS}
  ${controller_manager_INCLUDE_DIRS}
  ${wolf_hardware_interface_INCLUDE_DIRS}
  ${wolf_controller_utils_INCLUDE_DIRS}
  ${transmission_interface_INCLUDE_DIRS}
  ${gazebo_${ROS_VERSION}_control_INCLUDE_DIRS}
  ${realtime_tools_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS})

## Libraries
add_library(wolf_hw_sim src/${ROS_VERSION}/wolf_hw_sim.cpp)
target_link_libraries(wolf_hw_sim
    ${gazebo_ros_LIBRARIES}
    ${hardware_interface_LIBRARIES}
    ${controller_manager_LIBRARIES}
    ${wolf_hardware_interface_LIBRARIES}
    ${wolf_controller_utils_LIBRARIES}
    ${transmission_interface_LIBRARIES}
    ${gazebo_${ROS_VERSION}_control_LIBRARIES}
    ${realtime_tools_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${pluginlib_LIBRARIES}
)
target_compile_definitions(wolf_hw_sim PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")


install(
  DIRECTORY include/${ROS_VERSION}/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)
install(TARGETS wolf_hw_sim
    DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
)

install(FILES wolf_hw_sim_plugins.xml
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})


