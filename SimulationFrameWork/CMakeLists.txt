file(GLOB SIMULATION_FRAMEWORK_FILES
        *.h
        *.cpp
        src/*.h
        src/*.cpp
        utils/*.h
        utils/*.cpp
        )

if (USE_OpenMP)
    IF (OPENMP_FOUND)
        add_definitions(-DUSE_OPENMP)
    endif ()
endif ()

add_subdirectory(test)