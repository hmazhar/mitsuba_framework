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
    converter_ces
    converter_cloth
    converter_foam
    converter_dress
    converter_dress_press
    converter_viscosity
    converter_bucky
    converter_snowball
    converter_wave
    converter_drive
    converter_shaker
    converter_sls
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

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES}  ${ZLIB_LIBRARY} MitsubaGenerator)
    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES}  ${ZLIB_LIBRARY} MitsubaGenerator)

    INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

ENDFOREACH(PROGRAM)
