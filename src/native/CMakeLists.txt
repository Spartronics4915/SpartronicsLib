cmake_minimum_required(VERSION 3.9)

project(spartronicsnative VERSION 2019.1.0)

if(CROSS_COMPILE)
    set(CMAKE_SYSTEM_NAME Linux)
    set(CMAKE_SYSTEM_PROCESSOR arm)

    set(CMAKE_SYSROOT /home/devel/rasp-pi-rootfs)
    set(CMAKE_STAGING_PREFIX /home/devel/stage)

    set(tools /home/devel/gcc-4.7-linaro-rpi-gnueabihf)
    set(CMAKE_C_COMPILER ${tools}/bin/arm-linux-gnueabihf-gcc)
    set(CMAKE_CXX_COMPILER ${tools}/bin/arm-linux-gnueabihf-g++)

    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
endif()

# Find librealsense2 and jni
find_package(realsense2 REQUIRED)
find_package(JNI REQUIRED)
include_directories(${JNI_INCLUDE_DIRS})

# We only support C++ and above
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Add sources
add_library(${PROJECT_NAME} SHARED
    t265wrapper.hpp
    t265wrapper.cpp
)

# Link to librealsense2 and jni
target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
    ${JNI_LIBRARY} # This doesn't actually do anything
)

message(
    ${realsense2_LIBRARY}
    ${JNI_LIBRARY} 
)