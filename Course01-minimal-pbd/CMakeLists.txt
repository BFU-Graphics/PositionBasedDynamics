file(GLOB my_src src/*.cpp)

add_executable(minimal-pbd ${my_src})
target_link_libraries(minimal-pbd
        PUBLIC
        igl::core
        igl::opengl_glfw
        ${MY_DISCREGRID_LIBS}
        )