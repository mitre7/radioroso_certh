SET(OPENNI2_INCLUDE_SEARCH_PATHS $ENV{OPENNI2_INCLUDE})
SET(OPENNI2_LIB_SEARCH_PATHS $ENV{OPENNI2_REDIST})

SET(OPENNI2_INCLUDE_SEARCH_PATHS ${OPENNI2_INCLUDE_SEARCH_PATHS} "/usr/local/include" "/usr/include")
SET(OPENNI2_LIB_SEARCH_PATHS "${OPENNI2_LIB_SEARCH_PATHS}" "/usr/local/lib" "/usr/lib")

find_path(OPENNI2_INCLUDE_DIR NAMES OpenNI.h  HINTS ${OPENNI2_INCLUDE_SEARCH_PATHS})

# Finally the library itself
find_library(OPENNI2_LIBRARY NAMES OpenNI2 PATHS ${OPENNI2_LIB_SEARCH_PATHS})

IF ( OPENNI2_LIBRARY AND OPENNI2_INCLUDE_DIR )
set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY} )
set(OPENNI2_INCLUDE_DIRS ${OPENNI2_INCLUDE_DIR} )
ENDIF()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OPENNI2 DEFAULT_MSG OPENNI2_LIBRARY OPENNI2_INCLUDE_DIR)
mark_as_advanced(OPENNI2_INCLUDE_DIR OPENNI2_LIBRARY )