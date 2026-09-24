cmake_minimum_required(VERSION 3.16)
project(g1_audio_sdk_helper LANGUAGES CXX)

if(NOT DEFINED UNITREE_SDK_ROOT)
  message(FATAL_ERROR "UNITREE_SDK_ROOT must point to third_party/unitree_sdk2")
endif()

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory("${UNITREE_SDK_ROOT}" "${CMAKE_BINARY_DIR}/unitree_sdk2" EXCLUDE_FROM_ALL)

add_executable(g1_audio_sdk_helper g1_audio/sdk_helper.cpp)
target_link_libraries(g1_audio_sdk_helper PRIVATE unitree_sdk2)
target_compile_features(g1_audio_sdk_helper PRIVATE cxx_std_17)
set_target_properties(g1_audio_sdk_helper PROPERTIES
  BUILD_RPATH "\$ORIGIN/../lib"
  INSTALL_RPATH "\$ORIGIN/../lib"
)
target_link_options(g1_audio_sdk_helper PRIVATE -Wl,--disable-new-dtags)
