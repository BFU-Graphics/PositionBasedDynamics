file(GLOB SIMULATION_FRAMEWORK_FILES
        *.h *.cpp
        src/*.h src/*.cpp
        src/objects/*.h src/objects/*.cpp
        src/constraints/*.h src/constraints/*.cpp
        utils/*.h utils/*.cpp
        )

if (USE_OpenMP)
    IF (OPENMP_FOUND)
        add_definitions(-DUSE_OPENMP)
    endif ()
endif ()

add_subdirectory(test)