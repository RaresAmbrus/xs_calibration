cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(depth_calibration)

find_package(catkin REQUIRED COMPONENTS xs_data_reader)

set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)

add_definitions(-std=c++11 -O3)

# Find QT
find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED)
find_package(Boost REQUIRED)
include(${QT_USE_FILE})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(${Boost_LIBRARY_DIRS})
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS
   LIBRARIES sparse_gp
)

file(GLOB gp_headers include/sparse_gp/*.h include/sparse_gp/*.hpp)
file(GLOB gp_srcs src/sparse_gp/*.cpp)
message(STATUS ${gp_headers})
message(STATUS ${gp_srcs})
add_library(sparse_gp ${gp_headers} ${gp_srcs})
target_link_libraries(sparse_gp ${OpenCV_LIBS})

#add_library(helper_functions src/helper_functions.cpp include/helper_functions.h)
#target_link_libraries(helper_functions ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES})

add_executable(compute_disparity src/compute_disparity.cpp)
target_link_libraries(compute_disparity ${catkin_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} boost_system boost_filesystem)

add_executable(calibrate_depth_disparity src/calibrate_depth_disparity.cpp)
target_link_libraries(calibrate_depth_disparity sparse_gp ${catkin_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} boost_system boost_filesystem)
