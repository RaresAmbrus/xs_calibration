cmake_minimum_required(VERSION 2.8.3)
project(xs_data_reader)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs pcl_ros qt_build image_transport tf_conversions image_geometry)


set(CMAKE_CXX_FLAGS "-fopenmp -O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


link_directories(/opt/ros/hydro/lib/)

rosbuild_prepare_qt4(QtCore QtXml)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS
   LIBRARIES xs_data_reader
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include)

add_library(xs_data_reader src/helper_functions.cpp
                              include/helper_functions.h
                              )

add_dependencies(xs_data_reader ${catkin_EXPORTED_TARGETS})


 target_link_libraries(xs_data_reader
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
  )



add_executable(xs_data_reader_main src/xs_data_reader_main.cpp)

 target_link_libraries(xs_data_reader_main
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
    xs_data_reader
  )

