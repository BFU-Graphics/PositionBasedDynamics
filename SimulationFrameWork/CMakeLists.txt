file(GLOB SIMULATION_FRAMEWORK_FILES
        *.h
        *.cpp
        src/*.h
        src/*.cpp
        utils/*.h
        utils/*.cpp
        )

add_executable(test_sim test/main.cpp ${SIMULATION_FRAMEWORK_FILES} ${VISUALIZATION_FILES})
target_link_libraries(test_sim
        PUBLIC
        igl::core
        igl::opengl
        igl::opengl_glfw
        igl::opengl_glfw_imgui
        )
