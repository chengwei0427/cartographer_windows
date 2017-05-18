#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ceres" for configuration "Debug"
set_property(TARGET ceres APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(ceres PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/ceres-debug.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_DEBUG "D:\\commonLib\\dll-lib(vs2015)\\google-glog\\lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/ceres-debug.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS ceres )
list(APPEND _IMPORT_CHECK_FILES_FOR_ceres "${_IMPORT_PREFIX}/lib/ceres-debug.lib" "${_IMPORT_PREFIX}/bin/ceres-debug.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
