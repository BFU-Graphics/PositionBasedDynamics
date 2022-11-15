cmake_minimum_required(VERSION 3.15)
project(PositionBasedDynamics)

option(LIBIGL_OPENGL "Use OpenGL" ON)
option(LIBIGL_GLFW "Use GLFW" ON)
option(LIBIGL_IMGUI "Use ImGui" ON)
add_subdirectory(deps/libigl)

file(GLOB_RECURSE SIMULATION_FRAMEWORK_FILES SimulationFrameWork/*.cpp SimulationFrameWork/*.h SimulationFrameWork/*.inl.h)
file(GLOB_RECURSE RENDERING_FRAMEWORK_FILES RenderingFrameWork/*.cpp RenderingFrameWork/*.h RenderingFrameWork/*.inl.h)
add_library(HinaLib ${RENDERING_FRAMEWORK_FILES} ${SIMULATION_FRAMEWORK_FILES})
set_target_properties(HinaLib PROPERTIES CXX_STANDARD 20 CXX_EXTENSIONS ON)
target_link_libraries(HinaLib
        PUBLIC
        igl::core
        igl::opengl
        igl::glfw
        igl::imgui
        )
target_compile_definitions(HinaLib
        PUBLIC
        PBD_MODEL_DIR="${PROJECT_SOURCE_DIR}/Resources/models/"
        PBD_TEXTURE_DIR="${PROJECT_SOURCE_DIR}/Resources/textures/"
        )

set(PBD_EXAMPLES
        ClothSim01
        CubeSim01
        RigidSim01
        )

include_directories(
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/RenderingFrameWork
        ${CMAKE_SOURCE_DIR}/SimulationFrameWork
        deps/libigl/include
)

foreach (test ${PBD_EXAMPLES})
    add_executable(${test} Examples/${test}/main.cpp)
    set_target_properties(${test} PROPERTIES CXX_STANDARD 20 CXX_EXTENSIONS ON)
    target_link_libraries(${test}
            PUBLIC
            HinaLib
            )
endforeach ()

