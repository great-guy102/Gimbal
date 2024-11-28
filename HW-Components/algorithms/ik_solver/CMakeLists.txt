# 获取当前目录的库名
get_hwc_lib_name(${CMAKE_CURRENT_SOURCE_DIR} lib_name)
project(${lib_name} C CXX ASM)

get_hwc_lib_dir("${CMAKE_CURRENT_SOURCE_DIR}" lib_dir)
set(LIBRARY_OUTPUT_PATH ${lib_dir})

# 设置全局属性，用于传输文件信息
set_property(GLOBAL PROPERTY ${lib_name}_libs)
set_property(GLOBAL PROPERTY ${lib_name}_incs)

# 遍历子文件夹
file(
  GLOB children
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/*)
foreach(child ${children})
  set(sub_dir "${CMAKE_CURRENT_SOURCE_DIR}/${child}")

  # 如果子文件夹中存在 CMakeLists.txt
  if(IS_DIRECTORY ${sub_dir} AND EXISTS "${sub_dir}/CMakeLists.txt")
    get_hwc_lib_name(${sub_dir} sub_lib_name)

    message(STATUS "handling ${sub_dir}")
    # 执行子文件夹中的 CMakeLists.txt
    add_subdirectory(${child})

    # 获取子文件的库及头文件
    get_property(${sub_lib_name}_libs GLOBAL PROPERTY "${sub_lib_name}_libs")
    set_property(GLOBAL APPEND PROPERTY ${lib_name}_libs
                                        ${${sub_lib_name}_libs})

    get_property(${sub_lib_name}_incs GLOBAL PROPERTY "${sub_lib_name}_incs")
    set_property(GLOBAL APPEND PROPERTY ${lib_name}_incs
                                        ${${sub_lib_name}_incs})
  endif()
endforeach()
