# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Testing_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Testing_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Testing_FOUND FALSE)
  elseif(NOT Testing_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Testing_FOUND FALSE)
  endif()
  return()
endif()
set(_Testing_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Testing_FIND_QUIETLY)
  message(STATUS "Found Testing: 0.1.0 (${Testing_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Testing' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${Testing_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Testing_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${Testing_DIR}/${_extra}")
endforeach()
