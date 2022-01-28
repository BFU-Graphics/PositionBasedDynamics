cmake_minimum_required(VERSION 3.15)
project(PositionBasedDynamics)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(MY_EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/external)

find_package(MY_EIGEN REQUIRED)
find_package(MY_DISCREGRID)

#if (APPLE)
    # Libigl
    list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
    option(LIBIGL_WITH_OPENGL "Use OpenGL" ON)
    option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW" ON)
    include(libigl)
#endif ()

include_directories(${MY_EXTERNAL_DIR}/include)
link_directories(${MY_EXTERNAL_DIR}/bin external/lib)

add_definitions(
        -DPBD_MODEL_DIR="${CMAKE_SOURCE_DIR}/Resources/models/"
        -DPBD_TEXTURE_DIR="${CMAKE_SOURCE_DIR}/Resources/textures/"
)

add_subdirectory(Course01-minimal-pbd)