cmake_minimum_required(VERSION 3.15)
project(PositionBasedDynamics)

set(CMAKE_CXX_STANDARD 17)

OPTION (USE_OpenMP "Use OpenMP" ON)
IF(USE_OpenMP)
    FIND_PACKAGE(OpenMP)
    IF(OPENMP_FOUND)
        SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    ENDIF()
ENDIF()

if ( MSVC )
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W4")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4005 /wd4100 /wd4244 /wd4267 /wd4819 /wd4305 /wd4996 /wd4018 /wd4389 /wd4018 /wd4127 /wd4458 /wd4459 /wd4189 /wd4456 /wd4101 /wd4125 /wd4505")
endif()

# Set Cmake Variables
set(CMAKE_MSVC_RUNTIME_LIBRARY MultiThreaded$<$<CONFIG:Debug>:Debug>DLL)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/lib)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})


# Set My Variables
set(MY_EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/external) # default method
#if (APPLE)
set(MY_EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../Graphics-Dependencies/install) # method 2: READ CAREFULLY THIS LINE ~
#endif ()


# Find Packages
find_package(MY_EIGEN REQUIRED)
#find_package(MY_DISCREGRID REQUIRED)
#find_package(MY_OPENCV REQUIRED)


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
file(GLOB PBD_UTILS_FILES Utils/api/*.h Utils/api/*.cpp)
file(GLOB PBD_UTILS_FILES Utils/*.h Utils/*.cpp)
file(GLOB PBD_UTILS_VISUALIZATION_FILES Utils/visualization_src/*.h Utils/visualization_src/*.cpp)


file(GLOB VISUALIZATION_FILES
        RenderingFrameWork/*.h
        RenderingFrameWork/*.cpp
        RenderingFrameWork/src/*.h
        RenderingFrameWork/src/*.cpp
        )

# Global Definitions
add_definitions(
        -DPBD_MODEL_DIR="${CMAKE_SOURCE_DIR}/Resources/models/"
        -DPBD_TEXTURE_DIR="${CMAKE_SOURCE_DIR}/Resources/textures/"
        -DPBD_LOG_DIR="${CMAKE_SOURCE_DIR}/log/"
)


# Sub Targets
add_subdirectory(RenderingFrameWork)
add_subdirectory(SimulationFrameWork)
add_subdirectory(Course01-minimal-pbd)