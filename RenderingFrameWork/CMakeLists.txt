file(GLOB RENDERING_FRAMEWORK_FILES
        *.h
        *.cpp
        src/*.h
        src/*.cpp
        )

if (STATIC_RENDERING_LIB)
    add_library(HinaRender ${VISUALIZATION_FILES})
    target_link_libraries(HinaRender
            PUBLIC
            igl::core
            igl::opengl
            igl::opengl_glfw
            igl::opengl_glfw_imgui
            )
else (DYNAMIC_RENDERING_LIB)
    add_library(HinaRender SHARED ${VISUALIZATION_FILES})
    target_link_libraries(HinaRender
            PUBLIC
            igl::core
            igl::opengl
            igl::opengl_glfw
            igl::opengl_glfw_imgui
            )
endif ()

#add_executable(test_vis test/main.cpp ${RENDERING_FRAMEWORK_FILES})
#target_link_libraries(test_vis
#        PUBLIC
#        igl::core
#        igl::opengl
#        igl::opengl_glfw
#        igl::opengl_glfw_imgui
#        )