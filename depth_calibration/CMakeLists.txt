cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(depth_calibration)

set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)

add_definitions(-std=c++11 -O3)

# Find QT
find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED)
find_package(Boost REQUIRED)
include(${QT_USE_FILE})

link_directories(${Boost_LIBRARY_DIRS})
include_directories(include)

add_library(helper_functions src/helper_functions.cpp include/helper_functions.h)
target_link_libraries(helper_functions ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES})

add_executable(compute_disparity src/compute_disparity.cpp)
target_link_libraries(compute_disparity helper_functions ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} boost_system boost_filesystem)

add_executable(calibrate_depth_disparity src/calibrate_depth_disparity.cpp)
target_link_libraries(calibrate_depth_disparity helper_functions ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} boost_system boost_filesystem)
