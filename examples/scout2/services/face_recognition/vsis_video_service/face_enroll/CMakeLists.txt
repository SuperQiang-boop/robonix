cmake_minimum_required(VERSION 3.10)
project(face_enroll CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 仓库根: 归一化(吸收 /.. 字面量), 同时注入源码作运行时前缀
get_filename_component(ROOT_ABS "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(ROOT ${ROOT_ABS})
set(SDK ${ROOT}/third_party/inspireface_sdk)
set(JSON ${ROOT}/third_party/json)

# json 两种布局兼容: 仓库 include/ 或裸单头
if(EXISTS ${JSON}/include/nlohmann/json.hpp)
  set(JSON_INC ${JSON}/include)
else()
  set(JSON_INC ${JSON})
endif()
include_directories(${SDK}/include ${JSON_INC})

# 现役库: third_party/inspireface_sdk/lib/libInspireFace.so
# x86 = 自编 (官方解压包 glibc 不兼容); arm = 同位摆放 (官方包可用)
# 判例: 大写 libInspireFace.so → NAMES 必须带 InspireFace
find_library(ISF_LIB NAMES InspireFace inspireface PATHS ${SDK}/lib NO_DEFAULT_PATH)
if(NOT ISF_LIB)
  message(FATAL_ERROR "libInspireFace.so 没找到, 找过: ${SDK}/lib")
endif()
message(STATUS "ISF lib: ${ISF_LIB}")
message(STATUS "PROJECT_ROOT: ${ROOT_ABS}")

add_executable(face_enroll main.cpp)
target_compile_definitions(face_enroll PRIVATE PROJECT_ROOT="${ROOT_ABS}")
target_link_libraries(face_enroll ${ISF_LIB} pthread dl)
