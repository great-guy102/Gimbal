# 获取当前目录的库名
get_hwc_lib_name("${CMAKE_CURRENT_SOURCE_DIR}" lib_name)
project(${lib_name} C CXX ASM)

# 获取当前目录的库目录
get_hwc_lib_dir("${CMAKE_CURRENT_SOURCE_DIR}" lib_dir)
set(LIBRARY_OUTPUT_PATH ${lib_dir})

# 设置C++标准为C++17，并要求必须支持
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置全局属性
set_property(GLOBAL PROPERTY ${lib_name}_libs)
set_property(GLOBAL PROPERTY ${lib_name}_incs)

# 如果使用当前库，将通过 ${lib_name}_incs 传递 public 头文件，通过 ${lib_name}_libs 传递库名
if(use_${lib_name})
  # 设置公共和私有的头文件目录
  set(${lib_name}_public_includes ${CMAKE_CURRENT_SOURCE_DIR}
                                  ${CMAKE_CURRENT_SOURCE_DIR}/inc)
  set(${lib_name}_private_includes ${CMAKE_CURRENT_SOURCE_DIR}/prv)
  remove_redundant_include_directories(${lib_name}_public_includes)

  # 获取源文件
  file(GLOB_RECURSE ${lib_name}_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.c*)

  set(${lib_name}_file)

  # 设置依赖的库
  set(depend_libs drivers)


  # ~~~
  # 添加库并设置头文件目录 TODO: 后续 install 需要考虑 PUBLIC_HEADER 属性
  # ~~~
  add_library(${lib_name} STATIC ${${lib_name}_sources})

  target_include_directories(
    ${lib_name}
    PUBLIC ${${lib_name}_public_includes}
    PRIVATE ${${lib_name}_private_includes})

  # 对每个依赖的库，设置头文件目录，添加依赖和链接库
  foreach(depend_lib ${depend_libs})
    target_include_directories(
      ${lib_name}
      PUBLIC $<TARGET_PROPERTY:${depend_lib},PUBLIC_INCLUDE_DIRECTORIES>)
    target_include_directories(
      ${lib_name}
      INTERFACE $<TARGET_PROPERTY:${depend_lib},INTERFACE_INCLUDE_DIRECTORIES>
    )
    add_dependencies(${lib_name} ${depend_lib})
    target_link_libraries(${lib_name} PUBLIC ${depend_lib})
  endforeach()

  # 设置全局属性
  set_property(GLOBAL APPEND PROPERTY ${lib_name}_libs ${lib_name})
  set_property(GLOBAL APPEND PROPERTY ${lib_name}_incs
                                      ${${lib_name}_public_includes})
endif()
