cmake_minimum_required(VERSION 3.6)

# Set the default installation prefix if not set
if(NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin" CACHE PATH "Default install prefix" FORCE)
endif()

# Locate Azure Kinect SDK directories
set(K4A_DIR "C:/Program Files/Azure Kinect SDK v1.4.1")
set(K4ARECORD_DIR "C:/Program Files/Azure Kinect SDK v1.4.1/sdk/include/k4arecord")

# Language
enable_language(CXX)

# Compiler Settings
set(CMAKE_CXX_STANDARD 17) # require C++17 (or later) for filesystem
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler Option
set(FILESYSTEM)
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
  set(FILESYSTEM "stdc++fs")
elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  set(FILESYSTEM "c++fs")
endif()

# Project
project(preprocess_mkv LANGUAGES CXX)
add_executable(preprocess_mkv happly.h util.h kinect.hpp kinect.cpp main.cpp)

# (Option) Build with Static CRT
if(WIN32)
  option(BUILD_STATIC_CRT "build with static crt" OFF)
  if(BUILD_STATIC_CRT)
    set(CXX_FLAGS
      CMAKE_CXX_FLAGS
      CMAKE_CXX_FLAGS_DEBUG
      CMAKE_CXX_FLAGS_RELEASE
   )
    foreach(CXX_FLAG ${CXX_FLAGS})
      string(REPLACE "/MD" "/MT" ${CXX_FLAG} "${${CXX_FLAG}}")
    endforeach()
  endif()
endif()

# (Option) Start-Up Project for Visual Studio
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "preprocess_mkv")

# k4a
add_library(k4a::k4a SHARED IMPORTED)
set_target_properties(k4a::k4a PROPERTIES
  IMPORTED_LOCATION "${K4A_DIR}/sdk/windows-desktop/amd64/release/bin/k4a.dll"
  IMPORTED_IMPLIB "${K4A_DIR}/sdk/windows-desktop/amd64/release/lib/k4a.lib"
  INTERFACE_INCLUDE_DIRECTORIES "${K4A_DIR}/sdk/include"
)

# k4arecord
add_library(k4a::k4arecord SHARED IMPORTED)
set_target_properties(k4a::k4arecord PROPERTIES
  IMPORTED_LOCATION "${K4A_DIR}/sdk/windows-desktop/amd64/release/bin/k4arecord.dll"
  IMPORTED_IMPLIB "${K4A_DIR}/sdk/windows-desktop/amd64/release/lib/k4arecord.lib"
  INTERFACE_INCLUDE_DIRECTORIES "${K4A_DIR}/sdk/include"
)

# Find Package
find_package(OpenCV REQUIRED)
if(NOT WIN32)
  find_package(TBB REQUIRED)
endif()

# Set Package to Project
if(OpenCV_FOUND)
  target_link_libraries(preprocess_mkv k4a::k4a)
  target_link_libraries(preprocess_mkv k4a::k4arecord)
  target_link_libraries(preprocess_mkv ${OpenCV_LIBS})
  target_link_libraries(preprocess_mkv ${FILESYSTEM})
endif()

if(TBB_FOUND)
  target_link_libraries(preprocess_mkv TBB::tbb)
endif()

# Install rules
install(TARGETS preprocess_mkv
    RUNTIME DESTINATION .
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)