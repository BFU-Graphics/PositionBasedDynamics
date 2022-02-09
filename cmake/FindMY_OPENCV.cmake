if (MSVC)
    set(OpenCV_DIR ${MY_EXTERNAL_DIR}/x64/vc17/lib)
    if (NOT EXISTS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/opencv_core455.dll)
        file(GLOB MY_OPENCV_DLLS ${MY_EXTERNAL_DIR}/x64/vc17/bin/*.dll)
        file(INSTALL ${MY_OPENCV_DLLS} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    else()
        message("===== NO NEED TO RELOAD OPENCV =====")
    endif ()
endif ()
if (APPLE)
    set(OpenCV_DIR ${MY_EXTERNAL_DIR}/lib/cmake/opencv4)
    if (NOT EXISTS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/libopencv_core.4.5.5.dylib)
        file(GLOB MY_OPENCV_DYLIB ${MY_EXTERNAL_DIR}/lib/*.dylib)
        file(INSTALL ${MY_OPENCV_DYLIB} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    else()
        message("===== NO NEED TO RELOAD OPENCV =====")
    endif ()
endif ()
find_package(OpenCV REQUIRED)

set(MY_OPENCV_LIBRARY ${OpenCV_LIBS})
set(MY_OPENCV_LIBRARIES ${OpenCV_LIBS})
set(MY_OPENCV_LIB ${OpenCV_LIBS})
set(MY_OPENCV_LIBS ${OpenCV_LIBS})