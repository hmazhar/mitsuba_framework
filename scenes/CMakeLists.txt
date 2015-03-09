#=============================================================================
# CMake configuration file for the MitsubaGenerator library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#=============================================================================

# ------------------------------------------------------------------------------
# LIST THE FILES in the MitsubaGenerator LIBRARY
# ------------------------------------------------------------------------------

find_package(ZLIB)

SET(PARTIO_NAMES ${PARTIO_NAMES} partio)  
FIND_LIBRARY(PARTIO_LIBRARY NAMES ${PARTIO_NAMES} PATHS /usr/lib /usr/local/lib)

SET(MitsubaGenerator_SOURCES
    MitsubaGenerator.cpp
    )

SET(MitsubaGenerator_HEADERS
    MitsubaGenerator.h
    )


INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
    )

# ------------------------------------------------------------------------------
# ADD THE MitsubaGenerator LIBRARY
# ------------------------------------------------------------------------------

ADD_LIBRARY(MitsubaGenerator SHARED
    ${MitsubaGenerator_HEADERS}
    ${MitsubaGenerator_SOURCES})

SET_TARGET_PROPERTIES(MitsubaGenerator PROPERTIES
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_GPU}"
  COMPILE_DEFINITIONS "CH_API_COMPILE_UTILS")

TARGET_LINK_LIBRARIES(MitsubaGenerator )

INSTALL(TARGETS MitsubaGenerator
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    )


# ------------------------------------------------------------------------------
# Add Converter Programs
# ------------------------------------------------------------------------------

SET(CONVERTERS 
    scenes/converter_fording
)


FOREACH(PROGRAM ${CONVERTERS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_BUILDFLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} ${PARTIO_LIBRARY} ${ZLIB_LIBRARY} MitsubaGenerator)
    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES} ${PARTIO_LIBRARY} ${ZLIB_LIBRARY} MitsubaGenerator)

    INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

ENDFOREACH(PROGRAM)

