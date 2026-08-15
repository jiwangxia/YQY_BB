
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was PackageConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

#cudss-config.cmake
#
# Package COMPONENTs defined:
# cudss

# Imported interface targets provided:
#  * cudss        - shared library target
#  * cudss_static - static library target
#
# Output variables set:
#  * cudss_VERSION - Version of installed package
#  * cudss_INCLUDE_DIR - Location of  headers
#  * cudss_LIBRARY_DIR - Location of  libraries
#  * cudss_BINARY_DIR  - Location of  binaries

set(cudss_VERSION 0.8.0)

## Find header and lib directories via known header and lib names
set(_cudss_search_header_name cudss.h)
if(WIN32)
    set(_cudss_search_lib_name cudss.lib)
    set(_cudss_search_bin_name cudss64_0.dll)
else()
    set(_cudss_search_lib_name libcudss.so)
endif()

file(REAL_PATH "${CMAKE_CURRENT_LIST_DIR}" _cudss_cmake_config_realpath) # path to the location of cudss-config.cmake, with symlinks resolved
file(REAL_PATH "../../" _cudss_search_prefix BASE_DIRECTORY "${_cudss_cmake_config_realpath}")

if(WIN32)
    find_path(cudss_INCLUDE_DIR
              NAMES ${_cudss_search_header_name}
              PATHS ${_cudss_search_prefix}
              PATH_SUFFIXES
                ../include
                ../../include
              NO_DEFAULT_PATH
              NO_CACHE) # Set cache variable after path normalization

    find_path(cudss_LIBRARY_DIR
              NAMES ${_cudss_search_lib_name}
              PATHS ${_cudss_search_prefix}
              PATH_SUFFIXES
                ../lib
                ../lib/13
                ../../lib
                ../../lib/13
              DOC "Location of  libraries"
              NO_DEFAULT_PATH)

    find_path(cudss_BINARY_DIR
              NAMES ${_cudss_search_bin_name}
              PATHS ${_cudss_search_prefix}
              PATH_SUFFIXES
                ../bin
                ../bin/13
                ../../bin
                ../../bin/13
              DOC "Location of  binaries"
              NO_DEFAULT_PATH)
else()
    find_path(cudss_INCLUDE_DIR
              NAMES ${_cudss_search_header_name}
              PATHS ${_cudss_search_prefix}
              PATH_SUFFIXES
                ../include
                ../../include
                ../../../include
                ../../../../include # Debian/Ubuntu aarch64
              NO_DEFAULT_PATH
              NO_CACHE) # Set cache variable after path normalization

    find_path(cudss_LIBRARY_DIR
              NAMES ${_cudss_search_lib_name}
              PATHS ${_cudss_search_prefix}
              PATH_SUFFIXES
                ./
                ../lib
                ../lib/13
                ../lib64
                ../lib64/13
              DOC "Location of  libraries"
              NO_DEFAULT_PATH)

endif()

# Check headers and library directories are found
if(NOT EXISTS "${cudss_INCLUDE_DIR}")
    message(FATAL_ERROR "Header directory containing file ${_cudss_search_header_name} was not found relative to ${CMAKE_CURRENT_LIST_DIR}!")
endif()
if(NOT EXISTS "${cudss_LIBRARY_DIR}")
    message(FATAL_ERROR "Library directory containing file ${_cudss_search_lib_name} was not found relative to ${CMAKE_CURRENT_LIST_DIR}!")
endif()
if(WIN32 AND NOT EXISTS "${cudss_BINARY_DIR}")
    message(FATAL_ERROR "Binary directory containing file ${_cudss_search_bin_name} was not found relative to ${CMAKE_CURRENT_LIST_DIR}!")
endif()

# Normalize cudss_INCLUDE_DIR and set cache variable
get_filename_component(cudss_INCLUDE_DIR "${cudss_INCLUDE_DIR}" ABSOLUTE)
set(cudss_INCLUDE_DIR ${cudss_INCLUDE_DIR} CACHE STRING "Location of  headers")

unset(_cudss_search_header_name)
unset(_cudss_search_lib_name)
if (WIN32)
    unset(_cudss_search_bin_name)
endif()
unset(_cudss_search_prefix)

## Targets
include("${CMAKE_CURRENT_LIST_DIR}/cudss-targets.cmake")

## PackageConfig COMPONENTS
# Find installed components
set(_components)
set(cudss_cudss_FOUND 1)
list(APPEND _components cudss)

# The logic here is that CMake should look for the static library if
# cudss_static component is REQUIRED or
# cudss-static-targets.cmake are present
if(NOT WIN32)
    if(cudss_FIND_REQUIRED_cudss_static)
        if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/cudss-static-targets.cmake")
            if(NOT ${CMAKE_FIND_PACKAGE_NAME}_FIND_QUIETLY)
                message(STATUS "Detected and included cudss-static-targets.cmake")
            endif()
            include("${CMAKE_CURRENT_LIST_DIR}/cudss-static-targets.cmake")
            set(cudss_cudss_static_FOUND 1)
            list(APPEND _components cudss_static)
        else()
            message(FATAL_ERROR "cudss_static component is REQUIRED
but cudss-static-targets.cmake cannot be found")
        endif()
    else()
        if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/cudss-static-targets.cmake")
            if(NOT ${CMAKE_FIND_PACKAGE_NAME}_FIND_QUIETLY)
                message(STATUS "Detected and included cudss-static-targets.cmake")
            endif()
            include("${CMAKE_CURRENT_LIST_DIR}/cudss-static-targets.cmake"
                      OPTIONAL RESULT_VARIABLE _static_targets_found)
            if (NOT "${_static_targets_found}" STREQUAL "NOTFOUND")
                set(cudss_cudss_static_FOUND 1)
                list(APPEND _components cudss_static)
            endif()
        endif()
    endif()
endif()

#Allow Alias to targets
set_target_properties(cudss PROPERTIES IMPORTED_GLOBAL 1)
if(TARGET cudss_static)
    set_target_properties(cudss_static PROPERTIES IMPORTED_GLOBAL 1)
endif()

# Cleanup temporary variables and check if components satisfied
check_required_components(cudss)

## Report status
if(NOT ${CMAKE_FIND_PACKAGE_NAME}_FIND_QUIETLY)
    if (WIN32)
        message(STATUS
"Found cudss: (Version:${cudss_VERSION}
               CMakePackageDir:${CMAKE_CURRENT_LIST_DIR}
               IncludeDir:${cudss_INCLUDE_DIR}
               LibraryDir:${cudss_LIBRARY_DIR}
               BinaryDir:${cudss_BINARY_DIR}
               ComponentsFound:[${_components}])"
    )
    else()
        message(STATUS
"Found cudss: (Version:${cudss_VERSION}
               CMakePackageDir:${CMAKE_CURRENT_LIST_DIR}
               IncludeDir:${cudss_INCLUDE_DIR}
               LibraryDir:${cudss_LIBRARY_DIR}
               ComponentsFound:[${_components}])"
        )
    endif()
endif()
unset(_components)
