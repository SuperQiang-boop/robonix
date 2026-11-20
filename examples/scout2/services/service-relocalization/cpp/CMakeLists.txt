cmake_minimum_required(VERSION 3.16)
project(robonix_relocalization LANGUAGES C CXX)

option(RELOCALIZATION_BUILD_EXPORTER "Build the RTAB-Map snapshot exporter" ON)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
add_library(relocalization_bbs STATIC bbs.cpp registration.cpp)
set_target_properties(relocalization_bbs PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(relocalization_bbs PUBLIC cxx_std_17)
target_compile_options(relocalization_bbs PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(relocalization_bbs SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(relocalization_bbs PRIVATE Eigen3::Eigen ${PCL_LIBRARIES})
pybind11_add_module(_relocalization_bbs bindings.cpp registration_bindings.cpp)
target_link_libraries(_relocalization_bbs PRIVATE relocalization_bbs)
target_compile_options(_relocalization_bbs PRIVATE -Wall -Wextra -Wpedantic)

if(RELOCALIZATION_BUILD_EXPORTER)
    find_package(RTABMap 0.23.7 EXACT REQUIRED COMPONENTS core)
    add_executable(relocalization_export export_snapshot.cpp)
    target_compile_features(relocalization_export PRIVATE cxx_std_17)
    target_compile_options(relocalization_export PRIVATE -Wall -Wextra -Wpedantic)
    target_include_directories(relocalization_export SYSTEM PRIVATE ${RTABMap_INCLUDE_DIRS})
    target_link_libraries(relocalization_export PRIVATE ${RTABMap_LIBRARIES})
    install(TARGETS relocalization_export RUNTIME DESTINATION bin)
endif()

include(CTest)
if(BUILD_TESTING)
    add_test(NAME native_startup COMMAND ${Python3_EXECUTABLE}
        -m relocalization_rbnx.native_backend $<TARGET_FILE_DIR:_relocalization_bbs>)
    set_tests_properties(native_startup PROPERTIES ENVIRONMENT
        "PYTHONPATH=$<TARGET_FILE_DIR:_relocalization_bbs>:${CMAKE_CURRENT_SOURCE_DIR}/..")
    add_test(NAME bbs_search COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/backend_bbs.py)
    set_tests_properties(bbs_search PROPERTIES ENVIRONMENT
        "PYTHONPATH=$<TARGET_FILE_DIR:_relocalization_bbs>:${CMAKE_CURRENT_SOURCE_DIR}/..")
    add_test(NAME planar_registration COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/backend_registration.py)
    set_tests_properties(planar_registration PROPERTIES ENVIRONMENT
        "PYTHONPATH=$<TARGET_FILE_DIR:_relocalization_bbs>:${CMAKE_CURRENT_SOURCE_DIR}/..")
    add_test(NAME offline_matching COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/backend_matching.py)
    set_tests_properties(offline_matching PROPERTIES ENVIRONMENT
        "PYTHONPATH=$<TARGET_FILE_DIR:_relocalization_bbs>:${CMAKE_CURRENT_SOURCE_DIR}/..")
    if(RELOCALIZATION_BUILD_EXPORTER)
        add_executable(relocalization_snapshot_fixture ../tests/make_snapshot.cpp)
        target_compile_features(relocalization_snapshot_fixture PRIVATE cxx_std_17)
        target_include_directories(relocalization_snapshot_fixture SYSTEM PRIVATE ${RTABMap_INCLUDE_DIRS})
        target_link_libraries(relocalization_snapshot_fixture PRIVATE ${RTABMap_LIBRARIES})
        add_test(NAME snapshot_export COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/../tests/e2e_export.py
            $<TARGET_FILE:relocalization_snapshot_fixture> $<TARGET_FILE:relocalization_export>)
        set_tests_properties(snapshot_export PROPERTIES ENVIRONMENT
            "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/..;LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:$ENV{LD_LIBRARY_PATH}")
    endif()
endif()
