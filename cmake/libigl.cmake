if (TARGET igl::core)
    return()
endif ()

include(FetchContent)
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(FETCHCONTENT_BASE_DIR "${MY_EXTERNAL_DIR}/fetched-content-Debug")
else ()
    set(FETCHCONTENT_BASE_DIR "${MY_EXTERNAL_DIR}/fetched-content-Release")
endif ()
FetchContent_Declare(
        libigl
        GIT_REPOSITORY https://github.com/libigl/libigl.git
        GIT_TAG v2.3.0
)
FetchContent_GetProperties(libigl)
if (NOT libigl_POPULATED)
    FetchContent_Populate(libigl)
endif ()
list(PREPEND CMAKE_MODULE_PATH "${libigl_SOURCE_DIR}/cmake")
include(${libigl_SOURCE_DIR}/cmake/libigl.cmake)