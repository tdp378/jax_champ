# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_jax_behaviors_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED jax_behaviors_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(jax_behaviors_FOUND FALSE)
  elseif(NOT jax_behaviors_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(jax_behaviors_FOUND FALSE)
  endif()
  return()
endif()
set(_jax_behaviors_CONFIG_INCLUDED TRUE)

# output package information
if(NOT jax_behaviors_FIND_QUIETLY)
  message(STATUS "Found jax_behaviors: 0.0.0 (${jax_behaviors_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'jax_behaviors' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT jax_behaviors_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(jax_behaviors_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${jax_behaviors_DIR}/${_extra}")
endforeach()
