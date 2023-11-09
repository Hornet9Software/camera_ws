#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "camera_calibration_parsers::camera_calibration_parsers" for configuration ""
set_property(TARGET camera_calibration_parsers::camera_calibration_parsers APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(camera_calibration_parsers::camera_calibration_parsers PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "rclcpp::rclcpp;rcpputils::rcpputils"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcamera_calibration_parsers.so"
  IMPORTED_SONAME_NOCONFIG "libcamera_calibration_parsers.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS camera_calibration_parsers::camera_calibration_parsers )
list(APPEND _IMPORT_CHECK_FILES_FOR_camera_calibration_parsers::camera_calibration_parsers "${_IMPORT_PREFIX}/lib/libcamera_calibration_parsers.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
