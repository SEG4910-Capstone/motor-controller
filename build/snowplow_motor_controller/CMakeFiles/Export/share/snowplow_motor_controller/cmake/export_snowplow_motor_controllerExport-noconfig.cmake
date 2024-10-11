#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "snowplow_motor_controller::snowplow_motor_controller" for configuration ""
set_property(TARGET snowplow_motor_controller::snowplow_motor_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(snowplow_motor_controller::snowplow_motor_controller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsnowplow_motor_controller.so"
  IMPORTED_SONAME_NOCONFIG "libsnowplow_motor_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS snowplow_motor_controller::snowplow_motor_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_snowplow_motor_controller::snowplow_motor_controller "${_IMPORT_PREFIX}/lib/libsnowplow_motor_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
