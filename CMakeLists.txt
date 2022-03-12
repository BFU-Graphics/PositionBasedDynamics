cmake_minimum_required(VERSION 3.15)
project(PositionBasedDynamics)

OPTION(USE_OpenMP "Use OpenMP" ON)
OPTION(STATIC_RENDERING_LIB "Use Rendering Framework as static lib" OFF)
OPTION(DYNAMIC_RENDERING_LIB "Use Rendering Framework as static lib" OFF)

# Set Variables
set(MY_EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../Graphics-Dependencies/install) # Determine External Install Dir
set(CMAKE_CXX_STANDARD 17)
if (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4005 /wd4100 /wd4244 /wd4267 /wd4819 /wd4305 /wd4996 /wd4018 /wd4389 /wd4018 /wd4127 /wd4458 /wd4459 /wd4189 /wd4456 /wd4101 /wd4125 /wd4505")
    set(CMAKE_MSVC_RUNTIME_LIBRARY MultiThreaded$<$<CONFIG:Debug>:Debug>DLL)
endif ()
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/Debug)
endif ()
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/Release)
endif ()
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/lib)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

# Find Packages
if (USE_OpenMP)
    find_package(MY_OPENMP REQUIRED)
    if (OPENMP_FOUND)
        add_definitions(-DUSE_OPENMP)
    endif ()
endif ()

option(LIBIGL_WITH_OPENGL "Use OpenGL" ON)
option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW" ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui" ON)
include(libigl)

# Global Includes & Link Libraries
include_directories(${CMAKE_SOURCE_DIR} ${MY_EXTERNAL_DIR}/include)
link_directories(${MY_EXTERNAL_DIR}/bin ${MY_EXTERNAL_DIR}/lib)

# Global Definitions
add_definitions(
        -DPBD_MODEL_DIR="${CMAKE_SOURCE_DIR}/Resources/models/"
        -DPBD_TEXTURE_DIR="${CMAKE_SOURCE_DIR}/Resources/textures/"
)

file(GLOB_RECURSE RENDERING_FRAMEWORK_FILES
        ${CMAKE_SOURCE_DIR}/RenderingFrameWork/*.h
        ${CMAKE_SOURCE_DIR}/RenderingFrameWork/*.cpp
        )

file(GLOB_RECURSE SIMULATION_FRAMEWORK_FILES
        ${CMAKE_SOURCE_DIR}/SimulationFrameWork/*.h
        ${CMAKE_SOURCE_DIR}/SimulationFrameWork/*.cpp
        )

# Sub Targets
add_subdirectory(RenderingFrameWork)
add_subdirectory(SimulationFrameWork)
add_subdirectory(Examples)
