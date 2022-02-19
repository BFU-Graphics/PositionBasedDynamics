set(TEST_EXAMPLES
        TestCloth
        TestCube
        TestStarryFish
        TestRigidBody
        )

foreach (test ${TEST_EXAMPLES})
    if (STATIC_RENDERING_LIB)
        add_executable(${test} ${test}/main.cpp ${SIMULATION_FRAMEWORK_FILES})
        target_link_directories(test_sim
                PUBLIC
                ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
                )
        target_link_libraries(${test}
                PUBLIC
                igl::core
                HinaRender
                )
    elseif (DYNAMIC_RENDERING_LIB)
        add_executable(${test} ${test}/main.cpp ${SIMULATION_FRAMEWORK_FILES})
        target_link_directories(test_sim
                PUBLIC
                ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
                )
        target_link_libraries(${test}
                PUBLIC
                igl::core
                HinaRender
                )
    else ()
        add_executable(${test} ${test}/main.cpp ${SIMULATION_FRAMEWORK_FILES} ${VISUALIZATION_FILES})
        target_link_libraries(${test}
                PUBLIC
                igl::core
                igl::opengl
                igl::opengl_glfw
                igl::opengl_glfw_imgui
                )
    endif ()
endforeach ()