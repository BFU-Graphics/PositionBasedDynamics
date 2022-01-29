cmake_minimum_required(VERSION 3.15)
project(PositionBasedDynamics)


# Set Cmake Variables
set(CMAKE_MSVC_RUNTIME_LIBRARY MultiThreaded$<$<CONFIG:Debug>:Debug>DLL)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/lib)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})


# Set My Variables
set(MY_EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/external) # default method
if (APPLE)
    set(MY_EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../Graphics-Dependencies/install) # method 2: READ CAREFULLY THIS LINE ~
endif ()


# Find Packages
find_package(MY_EIGEN REQUIRED)
find_package(MY_DISCREGRID REQUIRED)
find_package(MY_OPENCV REQUIRED)


#if (APPLE)
# Libigl
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
option(LIBIGL_WITH_OPENGL "Use OpenGL" ON)
option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW" ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui" ON)
include(libigl)
#endif ()


# Global Includes & Link Libraries
include_directories(${CMAKE_SOURCE_DIR} ${MY_EXTERNAL_DIR}/include)
link_directories(${MY_EXTERNAL_DIR}/bin ${MY_EXTERNAL_DIR}/lib)


# Global Source Files
file(GLOB PBD_UTILS_FILES Utils/*.h Utils/*.cpp)
file(GLOB PBD_UTILS_VISUALIZATION_FILES Utils/visualization/*.h Utils/visualization/*.cpp)


# Global Definitions
add_definitions(
        -DPBD_MODEL_DIR="${CMAKE_SOURCE_DIR}/Resources/models/"
        -DPBD_TEXTURE_DIR="${CMAKE_SOURCE_DIR}/Resources/textures/"
)


# Sub Targets
add_subdirectory(Course01-minimal-pbd)