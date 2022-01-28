cmake_minimum_required(VERSION 3.15)
project(PositionBasedDynamics)

#if (APPLE)
    # Libigl
    list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
    option(LIBIGL_WITH_OPENGL "Use OpenGL" ON)
    option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW" ON)
    include(libigl)
#endif ()

add_definitions(
        -DPBD_MODEL_DIR="${CMAKE_SOURCE_DIR}/Resources/models/"
        -DPBD_TEXTURE_DIR="${CMAKE_SOURCE_DIR}/Resources/textures/"
)

add_subdirectory(Course01-minimal-pbd)
