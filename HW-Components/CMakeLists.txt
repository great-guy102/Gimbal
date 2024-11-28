cmake_minimum_required(VERSION 3.25)

project(hwcomponents)

set(HWC_LIB_PREFIX
  "${PROJECT_NAME}"
  CACHE INTERNAL "Library prefix for HWComponent")
set(HWC_WORKSPACE
  ${CMAKE_CURRENT_SOURCE_DIR}
  CACHE INTERNAL "Workspace path")

set(HWC_CMAKE_LIBRARY_PATH
  "${CMAKE_LIBRARY_PATH}/hwcomponents"
  CACHE INTERNAL "Path to HWComponent libraries")
set(HWC_CMAKE_MODULE_PATH
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
  CACHE INTERNAL "Path to HWComponent CMake modules")

include("${HWC_CMAKE_MODULE_PATH}/utils/tools.cmake")

set(lib_name "${HWC_LIB_PREFIX}")

set_property(GLOBAL PROPERTY ${lib_name}_libs)
set_property(GLOBAL PROPERTY ${lib_name}_incs)

set(${lib_name}_public_includes ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/Include)
file(GLOB ${lib_name}_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.c[^make]*)
remove_redundant_include_directories(${lib_name}_public_includes)

if(${lib_name}_public_includes)
  set_property(GLOBAL APPEND PROPERTY ${lib_name}_libs
    ${${lib_name}_public_includes})
endif()

if(${lib_name}_sources)
  set_property(GLOBAL APPEND PROPERTY ${lib_name}_srcs ${${lib_name}_sources})
  add_library(${lib_name} STATIC ${${lib_name}_sources})
  target_include_directories(${lib_name} PUBLIC ${${lib_name}_public_includes})
  set_property(GLOBAL APPEND PROPERTY ${lib_name}_libs ${lib_name})
endif()

file(
  GLOB children
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/*)

foreach(child ${children})
  set(sub_dir "${CMAKE_CURRENT_SOURCE_DIR}/${child}")

  if(IS_DIRECTORY ${sub_dir} AND EXISTS "${sub_dir}/CMakeLists.txt")
    get_hwc_lib_name(${sub_dir} sub_lib_name)

    message(STATUS "handling ${sub_dir}")
    add_subdirectory(${child})

    get_property(${sub_lib_name}_libs GLOBAL PROPERTY "${sub_lib_name}_libs")
    set_property(GLOBAL APPEND PROPERTY ${lib_name}_libs
      ${${sub_lib_name}_libs})

    get_property(${sub_lib_name}_incs GLOBAL PROPERTY "${sub_lib_name}_incs")
    # include_directories(${${sub_lib_name}_incs})
    set_property(GLOBAL APPEND PROPERTY ${lib_name}_incs
      ${${sub_lib_name}_incs})
  endif()
endforeach()

get_property(${lib_name}_libs GLOBAL PROPERTY "${lib_name}_libs")
set(${lib_name}_libs
  ${${lib_name}_libs}
  CACHE INTERNAL "libraries of ${PROJECT_NAME}")
message(STATUS "libraries of ${PROJECT_NAME}:")

foreach(item ${${lib_name}_libs})
  message(STATUS "- ${item}")
endforeach()

get_property(${lib_name}_incs GLOBAL PROPERTY "${lib_name}_incs")
set(${lib_name}_incs
  ${${lib_name}_incs}
  CACHE INTERNAL "the include directories of ${PROJECT_NAME}")
# message(STATUS "the include directories of ${PROJECT_NAME}:")
# foreach(item ${${lib_name}_incs})
#   message(STATUS "- ${item}")
# endforeach()
