cmake_minimum_required(VERSION 3.16)
project(g1_arm_action_cli LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(UNITREE_SDK2_DIR "" CACHE PATH "Path to an official unitree_sdk2 checkout")
if(NOT UNITREE_SDK2_DIR OR NOT EXISTS "${UNITREE_SDK2_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "Set UNITREE_SDK2_DIR to an official unitree_sdk2 checkout")
endif()

set(BUILD_EXAMPLES OFF CACHE BOOL "Never build Unitree motion examples" FORCE)
add_subdirectory("${UNITREE_SDK2_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/unitree_sdk2"
                 EXCLUDE_FROM_ALL)

add_executable(g1_arm_action_cli src/g1_arm_action_cli.cpp)
target_link_libraries(g1_arm_action_cli PRIVATE unitree_sdk2)
target_compile_options(g1_arm_action_cli PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(g1_arm_action_cli PROPERTIES
  INSTALL_RPATH "\$ORIGIN/../lib"
)
target_link_options(g1_arm_action_cli PRIVATE -Wl,--disable-new-dtags)

install(TARGETS g1_arm_action_cli RUNTIME DESTINATION bin)

set(UNITREE_DDS_LIBRARY_DIR
    "${UNITREE_SDK2_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}")
foreach(UNITREE_DDS_LIBRARY
        libddsc.so libddsc.so.0 libddscxx.so libddscxx.so.0)
  if(NOT EXISTS "${UNITREE_DDS_LIBRARY_DIR}/${UNITREE_DDS_LIBRARY}")
    message(FATAL_ERROR
      "Required Unitree DDS library is missing: ${UNITREE_DDS_LIBRARY}")
  endif()
  install(FILES "${UNITREE_DDS_LIBRARY_DIR}/${UNITREE_DDS_LIBRARY}"
          DESTINATION lib)
endforeach()

install(FILES "${UNITREE_SDK2_DIR}/LICENSE"
        DESTINATION share/licenses/unitree_sdk2)
install(FILES "${UNITREE_SDK2_DIR}/licenses/eclipse-cyclonedds/cyclonedds/LICENSE"
        DESTINATION share/licenses/unitree_sdk2/cyclonedds)
install(FILES "${UNITREE_SDK2_DIR}/licenses/eclipse-cyclonedds/cyclonedds-cxx/LICENSE"
        DESTINATION share/licenses/unitree_sdk2/cyclonedds-cxx)
install(FILES "${UNITREE_SDK2_DIR}/licenses/eclipse-iceoryx/iceoryx/LICENSE"
        DESTINATION share/licenses/unitree_sdk2/iceoryx)
install(FILES "${UNITREE_SDK2_DIR}/licenses/Tencent/rapidjson/LICENSE"
        DESTINATION share/licenses/unitree_sdk2/rapidjson)

