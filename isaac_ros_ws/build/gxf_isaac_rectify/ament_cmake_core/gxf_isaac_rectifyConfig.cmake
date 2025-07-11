# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_gxf_isaac_rectify_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED gxf_isaac_rectify_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(gxf_isaac_rectify_FOUND FALSE)
  elseif(NOT gxf_isaac_rectify_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(gxf_isaac_rectify_FOUND FALSE)
  endif()
  return()
endif()
set(_gxf_isaac_rectify_CONFIG_INCLUDED TRUE)

# output package information
if(NOT gxf_isaac_rectify_FIND_QUIETLY)
  message(STATUS "Found gxf_isaac_rectify: 3.2.5 (${gxf_isaac_rectify_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'gxf_isaac_rectify' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${gxf_isaac_rectify_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(gxf_isaac_rectify_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${gxf_isaac_rectify_DIR}/${_extra}")
endforeach()
