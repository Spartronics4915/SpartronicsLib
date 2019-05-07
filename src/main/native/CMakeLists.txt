cmake_minimum_required(VERSION 3.1.0)

project(spartronicslib)

# Find librealsense2 and jni
find_package(realsense2 REQUIRED)
find_package(JNI REQUIRED)

# We only support C++ and above
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Add sources
add_library(${PROJECT_NAME} SHARED t265wrapper.cpp)
add_library(${PROJECT_NAME} SHARED t265wrapper.h)

# Link librealsense2 to the target
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
