if (MSVC)
    set(MY_DISCREGRID_LIBRARIE Discregrid$<$<CONFIG:Debug>:_d>)
    set(MY_DISCREGRID_LIBRARIES Discregrid$<$<CONFIG:Debug>:_d>)
    set(MY_DISCREGRID_LIB Discregrid$<$<CONFIG:Debug>:_d>)
    set(MY_DISCREGRID_LIBS Discregrid$<$<CONFIG:Debug>:_d>)
endif ()
if (APPLE)
    set(MY_DISCREGRID_LIBRARIE Discregrid)
    set(MY_DISCREGRID_LIBRARIES Discregrid)
    set(MY_DISCREGRID_LIB Discregrid)
    set(MY_DISCREGRID_LIBS Discregrid)
endif ()
