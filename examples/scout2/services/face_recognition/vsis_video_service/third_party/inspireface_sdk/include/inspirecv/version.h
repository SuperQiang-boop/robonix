#ifndef INSPIRECV_VERSION_H
#define INSPIRECV_VERSION_H

#ifndef INSPIRECV_API
#define INSPIRECV_API
#endif

namespace inspirecv {

const char* INSPIRECV_API GetVersion();

const char* INSPIRECV_API GetCVBackend();

// One process-wide description of the complete InspireCV library. Version,
// backend, toolchain and optional acceleration information intentionally live
// together so Image and Task never report separate build identities.
struct LibraryInfo {
    int version_major;
    int version_minor;
    int version_patch;
    const char* version_string;
    const char* build_date;
    const char* cv_backend;
    const char* system_name;          // e.g. Darwin, Linux, Windows
    const char* system_processor;     // e.g. arm64, x86_64
    const char* compiler_name;        // e.g. AppleClang, GNU, MSVC
    const char* compiler_version;     // compiler version string
    const char* build_type;           // Debug/Release/RelWithDebInfo
    int         cxx_standard;         // e.g. 11/14/17
    bool        lto_enabled;
    bool        sse_enabled;
    bool        avx2_enabled;
    bool        neon_enabled;
    bool        task_tiling_enabled;
    int         task_tile_width;
    const char* cxx_flags;
};

const LibraryInfo& INSPIRECV_API GetLibraryInfo();

void INSPIRECV_API PrintLibraryInfo();

}  // namespace inspirecv

#endif  // INSPIRECV_VERSION_H
