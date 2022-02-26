include_directories(${CMAKE_SOURCE_DIR}/RenderingFrameWork)

if (STATIC_RENDERING_LIB)
    add_library(HinaRender ${RENDERING_FRAMEWORK_FILES})
    target_link_libraries(HinaRender
            PUBLIC
            igl::core
            igl::opengl
            igl::opengl_glfw
            igl::opengl_glfw_imgui
            )
elseif (DYNAMIC_RENDERING_LIB)
    add_library(HinaRender SHARED ${RENDERING_FRAMEWORK_FILES})
    target_link_libraries(HinaRender
            PUBLIC
            igl::core
            igl::opengl
            igl::opengl_glfw
            igl::opengl_glfw_imgui
            )
else ()
endif ()
