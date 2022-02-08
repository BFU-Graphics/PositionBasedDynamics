file(GLOB COURSE01_SRC src/*.cpp)

add_executable(minimal-pbd ${COURSE01_SRC} ${PBD_UTILS_FILES} ${PBD_UTILS_VISUALIZATION_FILES})
target_link_libraries(minimal-pbd
        PUBLIC
        igl::core
        igl::opengl
        igl::opengl_glfw
        igl::opengl_glfw_imgui
        ${MY_DISCREGRID_LIBS}
        ${MY_OPENCV_LIBS}
        )
