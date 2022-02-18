file(GLOB SIMULATION_FRAMEWORK_FILES
        *.h
        *.cpp
        src/*.h
        src/*.cpp
        utils/*.h
        utils/*.cpp
        )

if (STATIC_RENDERING_LIB)
    add_executable(test_sim test/main.cpp ${SIMULATION_FRAMEWORK_FILES})
    target_link_directories(test_sim
            PUBLIC
            ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
            )
    target_link_libraries(test_sim
            PUBLIC
            igl::core
            HinaRender
            )
elseif (DYNAMIC_RENDERING_LIB)
    add_executable(test_sim test/main.cpp ${SIMULATION_FRAMEWORK_FILES})
    target_link_directories(test_sim
            PUBLIC
            ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
            )
    target_link_libraries(test_sim
            PUBLIC
            igl::core
            HinaRender
            )
else ()
    add_executable(test_sim test/main.cpp ${SIMULATION_FRAMEWORK_FILES} ${VISUALIZATION_FILES})
    target_link_libraries(test_sim
            PUBLIC
            igl::core
            igl::opengl
            igl::opengl_glfw
            igl::opengl_glfw_imgui
            )
endif ()
if (USE_OpenMP)
    IF (OPENMP_FOUND)
        add_definitions(-DUSE_OPENMP)
    endif ()
endif ()