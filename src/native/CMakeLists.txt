# Does not support Windows for cross compilation
# Make sure you've run the installRoboRioToolchain Gradle task before running
# If you want to cross compile do -DCMAKE_TOOLCHAIN_FILE=roborio_toolchain.cmake

cmake_minimum_required(VERSION 3.9)

project(spartronicsnative VERSION 2019.1.0)

# Find librealsense2 and jni
find_package(realsense2 REQUIRED)
if (NOT CMAKE_TOOLCHAIN_FILE)
    find_package(JNI REQUIRED)
    include_directories(${JNI_INCLUDE_DIRS})
else()
    include_directories("jni/")
endif()

# We only support C++17 and above
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Add sources
add_library(${PROJECT_NAME} SHARED
    t265wrapper.hpp
    t265wrapper.cpp
)

# Link to librealsense2 and jni
target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
)
