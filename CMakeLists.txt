cmake_minimum_required(VERSION 3.0.0)
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

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  controller_manager
  wolf_hardware_interface
  transmission_interface
  pluginlib
  gazebo_ros
  gazebo_ros_control
  realtime_tools
  urdf)

# Depend on system installation of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  LIBRARIES wolf_hw_sim
  CATKIN_DEPENDS wolf_hardware_interface
  INCLUDE_DIRS include)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS})

## Libraries
add_library(wolf_hw_sim src/wolf_hw_sim.cpp)
target_link_libraries(wolf_hw_sim ${catkin_LIBRARIES}
                                 ${GAZEBO_LIBRARIES}
                      )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS wolf_hw_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES wolf_hw_sim_plugins.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

