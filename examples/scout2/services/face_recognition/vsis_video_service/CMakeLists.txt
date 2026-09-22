cmake_minimum_required(VERSION 3.16)
project(VsisVideoService)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 可执行文件输出到源码树 bin/ (与 audio 蓝本一致)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# ============================================================================
# 第三方库 —— 全部取自 third_party/
# ============================================================================

# 1. spdlog (header-only)
include_directories(${PROJECT_SOURCE_DIR}/third_party/spdlog/include)

# 2. nlohmann/json (header-only)
include_directories(${PROJECT_SOURCE_DIR}/third_party/json/include)

# 3. cpp-httplib (header-only; 纯 http, 不定义 CPPHTTPLIB_OPENSSL_SUPPORT, 无需 OpenSSL)
include_directories(${PROJECT_SOURCE_DIR}/third_party/httplib)

# 4. yaml-cpp (需编译库, 配置文件解析)
set(YAML_CPP_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS    OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB  OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_APPS     OFF CACHE BOOL "" FORCE)
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/yaml-cpp)

# 5. sqlite3 (person/roster.db 名单库)
find_package(SQLite3 REQUIRED)          # ubuntu: apt install libsqlite3-dev
message(STATUS "SQLite3 = ${SQLite3_VERSION}")

# 6. OpenCV (摄像头采集; 不带 highgui → 无头服务, 不链 GUI)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
message(STATUS "OpenCV = ${OpenCV_VERSION}")

# 9. InspireFace SDK (v1.2.3 MNN CPU-Only 产物形态)
set(ISF_SDK_DIR ${PROJECT_SOURCE_DIR}/third_party/inspireface_sdk)
set(ISF_INCLUDE_DIRS "")
foreach(_d include include/inspireface include/inspireface/c_api)
    if(EXISTS ${ISF_SDK_DIR}/${_d})
        list(APPEND ISF_INCLUDE_DIRS ${ISF_SDK_DIR}/${_d})
    endif()
endforeach()
find_library(ISF_LIB NAMES InspireFace 
             PATHS ${ISF_SDK_DIR}/lib ${ISF_SDK_DIR} NO_DEFAULT_PATH
             REQUIRED)
message(STATUS "InspireFace lib = ${ISF_LIB}")
add_library(inspireface SHARED IMPORTED)
set_target_properties(inspireface PROPERTIES
    IMPORTED_LOCATION ${ISF_LIB}
    INTERFACE_INCLUDE_DIRECTORIES "${ISF_INCLUDE_DIRS}")

# ============================================================================
# 构建模式
# ============================================================================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)     # 不指定时默认 Debug
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" _BUILD_TYPE_UP)
if(_BUILD_TYPE_UP STREQUAL "RELEASE")
    add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO)    # 发布裁到 INFO
else()
    add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)   # 开发全量
endif()

# ============================================================================
# 源文件
# ============================================================================
set(SOURCES
    main.cpp
    VideoServiceManager.cpp
    VideoServiceAuxer.cpp
    VideoCameraReader.cpp      # M2: OpenCV 采集线程
    FaceRecognitionEngine.cpp  # M3/M4: InspireFace 人脸识别引擎
    # VideoPersonStore.cpp       # M5: roster.db 名单库
    # VideoHttpApi.cpp           # M5: httplib 服务出口
)
set(HEADERS
    VideoDef.h
    VideoServiceManager.h
    VideoServiceAuxer.h
    VideoCameraReader.h
    FaceRecognitionEngine.h
    # VideoPersonStore.h
    # VideoHttpApi.h
)

add_executable(vsis_video_service ${SOURCES} ${HEADERS})

# 运行期定位 bin/ 下的 libinspireface.so (POST_BUILD 会拷过来)
set_target_properties(vsis_video_service PROPERTIES BUILD_RPATH "\$ORIGIN")

target_include_directories(vsis_video_service PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${SQLite3_INCLUDE_DIRS}
)

target_link_libraries(vsis_video_service PUBLIC
    inspireface              # InspireFace (IMPORTED)
    ${OpenCV_LIBS}           # opencv_core imgproc videoio
    ${SQLite3_LIBRARIES}     # sqlite3
    yaml-cpp::yaml-cpp
    pthread dl
)

# 把 libinspireface.so 拷到 bin/ 与可执行文件同目录, 配合 $ORIGIN rpath 开箱即跑
add_custom_command(TARGET vsis_video_service POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${ISF_LIB} $<TARGET_FILE_DIR:vsis_video_service>)