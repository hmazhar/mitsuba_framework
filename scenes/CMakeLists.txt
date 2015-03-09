#=============================================================================
# CMake configuration file for the MitsubaGenerator library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#=============================================================================

# ------------------------------------------------------------------------------
# Add Converter Programs
# ------------------------------------------------------------------------------

SET(CONVERTERS 
    converter_fording
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
