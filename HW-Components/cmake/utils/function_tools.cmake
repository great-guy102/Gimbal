function(get_format_folder_name folder res)
  get_filename_component(folder_name ${folder} NAME)
  string(TOLOWER ${folder_name} folder_name)
  set(${res}
      ${folder_name}
      PARENT_SCOPE)
  return()
endfunction()

function(get_sub_folder root_folder res)
  file(
    GLOB children
    RELATIVE ${root_folder}
    ${root_folder}/*)
  set(sub_folders)
  foreach(child ${children})
    set(sub_dir ${root_folder}/${child})
    if(IS_DIRECTORY ${sub_dir})
      list(APPEND sub_folders ${sub_dir})
    endif()
  endforeach()

  set(${res}
      ${sub_folders}
      PARENT_SCOPE)
  return()
endfunction()

function(get_hwc_lib_tail lib_folder res)
  file(
    GLOB re_dir
    RELATIVE ${HWC_WORKSPACE}
    ${lib_folder})
  string(TOLOWER ${re_dir} re_dir)
  string(REPLACE "/" "_" lib_tail ${re_dir})
  set(${res}
      ${lib_tail}
      PARENT_SCOPE)
endfunction()

function(get_hwc_lib_name lib_folder res)
  get_hwc_lib_tail(${lib_folder} lib_tail)
  set(${res}
      "${HWC_LIB_PREFIX}_${lib_tail}"
      PARENT_SCOPE)
endfunction()

function(get_hwc_lib_dir lib_folder res)
  file(
    GLOB re_dir
    RELATIVE ${HWC_WORKSPACE}
    ${lib_folder})
  set(${res}
      "${HWC_CMAKE_LIBRARY_PATH}/${re_dir}"
      PARENT_SCOPE)
endfunction()

function(remove_redundant_include_directories list_value)
  set(clean_list)
  foreach(dir ${${list_value}})
    file(
      GLOB headers
      RELATIVE ${dir}
      ${dir}/*.h*)

    if(headers)
      list(APPEND clean_list ${dir})
    endif()
  endforeach()
  set(${list_value}
      ${clean_list}
      PARENT_SCOPE)
  return()
endfunction()

# 递归包含头文件的函数
function(include_sub_directories_recursively root_dir)
  if(IS_DIRECTORY ${root_dir}) # 当前路径是一个目录吗，是的话就加入到包含目录
    message(STATUS "include dir: " ${root_dir})
    include_directories(${root_dir})
  endif()

  file(
    GLOB ALL_SUB
    RELATIVE ${root_dir}
    ${root_dir}/*) # 获得当前目录下的所有文件，让如ALL_SUB列表中
  foreach(sub ${ALL_SUB})
    if(IS_DIRECTORY ${root_dir}/${sub})
      include_sub_directories_recursively(${root_dir}/${sub}) # 对子目录递归调用，包含
    endif()
  endforeach()
endfunction()

function(search_incs_recurse root_dir res_list)
  get_filename_component(root_name ${root_dir} NAME)
  set(_${root_name}_incs ${root_dir})
  file(
    GLOB children
    RELATIVE ${root_dir}
    ${root_dir}/*)
  foreach(child ${children})
    set(sub_dir ${root_dir}/${child})
    if(IS_DIRECTORY ${sub_dir})
      get_filename_component(child_name ${child} NAME)
      set(__${child_name}_incs)
      search_incs_recurse(${sub_dir} __${child_name}_incs)
      list(APPEND _${root_name}_incs ${__${child_name}_incs})
    endif()
  endforeach()
  set(${res_list}
      ${_${root_name}_incs}
      PARENT_SCOPE)
  return()
endfunction()

macro(COMMON_LIB_HEADER)
  get_filename_component(current_folder ${CMAKE_CURRENT_SOURCE_DIR} NAME)
  string(TOLOWER ${current_folder} current_folder)
  file(RELATIVE_PATH currentRel ${HW_COMPONENTS_DIR}
       ${CMAKE_CURRENT_SOURCE_DIR})
  string(TOLOWER ${currentRel} currentRel)
  string(REPLACE "/" "_" lib_tail ${currentRel})

  project(${current_folder})

  set(lib_name ${HW_COMPONENTS_LIB_PREFIX}_${lib_tail})

  message(STATUS "handle lib ${lib_name}")

  set(LIBRARY_OUTPUT_PATH ${HW_COMPONENTS_LIB_DIR})
endmacro()

macro(MERGE_STATIC_LIBS lib_dir target_lib_name libs incs)
  set(_lib ${lib_dir}/lib${target_lib_name}.a)

  set(_commands)
  set(_depends)
  message(STATUS "these libs will be merged into ${_lib}:")

  # 遍历libs，将每一个lib添加到_commands中，并记录到_depends中
  foreach(lib ${${libs}})
    message(STATUS "    ${lib}")
    list(APPEND _commands $<TARGET_FILE:${lib}>)
    list(APPEND _depends ${lib})
  endforeach()

  # 判断当前操作系统是否是苹果系统
  if(APPLE)
    # 如果是苹果系统，则使用libtool将_commands中的每一个lib合并到_lib中
    add_custom_target(
      _${target_lib_name}
      COMMAND libtool -static ${_lib} ${_commands}
      WORKING_DIRECTORY ${lib_dir}
      DEPENDS ${depends})
  else()
    # 如果不是苹果系统，则使用ar将_commands中的每一个lib合并到_lib中
    add_custom_target(
      _${target_lib_name}
      COMMAND ar crsT ${_lib} ${_commands}
      WORKING_DIRECTORY ${lib_dir}
      DEPENDS ${depends})
  endif()

  # 创建一个静态库，用于存储合并后的lib
  add_library(${target_lib_name} STATIC IMPORTED GLOBAL)

  # 将_${target_lib_name}添加到target_lib_name的依赖中
  add_dependencies(${target_lib_name} _${target_lib_name})

  # 设置target_lib_name的属性，用于指定合并后的lib的位置
  set_target_properties(${target_lib_name} PROPERTIES IMPORTED_LOCATION ${_lib})

  target_include_directories(${target_lib_name} INTERFACE ${${incs}})
  # foreach(dep ${_depends}) target_link_libraries(${target_lib_name} INTERFACE
  # ${dep}) endforeach()

endmacro()

macro(ADD_ALL_SUBDIRECTORY curdir)
  file(
    GLOB children
    RELATIVE ${curdir}
    ${curdir}/*)
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child} AND EXISTS
                                           ${curdir}/${child}/CMakeLists.txt)
      add_subdirectory("${child}")
    endif()
  endforeach()
endmacro()

macro(INCLUDE_ALL_DIR curdir)
  file(
    GLOB children
    RELATIVE ${curdir}
    ${curdir}/*)

  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child} AND EXISTS ${curdir}/${child}/info.cmake)
      set(SUB_DIR ${child})
      include(${curdir}/${child}/info.cmake)
    endif()
  endforeach()
endmacro()

macro(SUB_ADD_SRC src)
  foreach(item ${${src}})
    list(APPEND ${PROJECT_NAME}_SOURCES ${item})
  endforeach()
endmacro()

macro(SUB_ADD_INC inc)
  foreach(item ${${inc}})
    list(APPEND ${PROJECT_NAME}_INCLUDES ${item})
  endforeach()
endmacro()

macro(CHECK_SUB_ENABLE enable prefix)
  set(${enable} ${${CONFIG_PREFIX}${prefix}-${SUB_DIR}})
  message(${CONFIG_PREFIX}${prefix}-${SUB_DIR}\ ${${enable}})
endmacro()
