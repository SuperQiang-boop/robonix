/**
 * @file VideoDef.h
 * @brief 全局常量定义
 *
 * 只放编译期不变量 (退出码/版本号等);
 * 一切可调参数进 config/cfg.yaml, 不在此处
 *
 * @author Teate
 * @date 2026-07-01
 * @version 1.1.0
 * @copyright Copyright (c) 2023, VSISLab. All rights reserved.
 */
#ifndef VSIS_VIDEO_DEF_H
#define VSIS_VIDEO_DEF_H

namespace Vsis {

// 进程退出码 (0 保留给 "成功", 失败一律非 0)
namespace exitcode {
    constexpr int OK            = 0;
    constexpr int FAIL_CONFIG   = 1;   // 目录定位/配置文件/日志装配失败
    constexpr int FAIL_CAMERA   = 2;   // 摄像头采集
    constexpr int FAIL_ISF      = 3;   // InspireFace
    // 后续: FAIL_CAMERA = 3, FAIL_DB = 4 ...
}

namespace facepath {
    // 全部相对工程根, 经 Auxer 拼接, 目录约定不进 cfg
    constexpr const char* PHOTO   = "person/photo/";
    constexpr const char* DB      = "person/db/face_hub.db";
    constexpr const char* MAPPING = "person/mapping.json";
    constexpr const char* PACK    = "third_party/inspireface_sdk/pack/Pikachu";
    constexpr int MIN_FACE_PX = 112;   // 算法契约, 非调参
}


} // namespace Vsis

#endif // VSIS_VIDEO_DEF_H
