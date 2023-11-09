#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "image_transport::image_transport" for configuration ""
set_property(TARGET image_transport::image_transport APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(image_transport::image_transport PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libimage_transport.so"
  IMPORTED_SONAME_NOCONFIG "libimage_transport.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS image_transport::image_transport )
list(APPEND _IMPORT_CHECK_FILES_FOR_image_transport::image_transport "${_IMPORT_PREFIX}/lib/libimage_transport.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
