// face_enroll v1.1 — 人脸注册工具 (add / list / del)
// v1.0→v1.1:
//   ★ 路径全改 CMake 注入 PROJECT_ROOT 拼接 (v1.0 硬编码 /vsis_video_service/... 是 arm 布局,
//     x86 仓库在 /root/vsis_video_service → pack ENOENT → SDK abort 核心转储;
//     probe 因运行时 HOME 推导幸免, arm 因硬编码撞对幸免)
//   ② add 不再拷贝照片: 使用者把照片放进 person/photo/, 工具只登记 (原判例④⑤废止)
//   ③ del 连带删除照片文件 (仍被其他条目引用则保留; 不在档仅提示)
//   ④ 首跑自动建 person/ person/db person/photo (v1.0 漏 db 目录, 新机首跑 hub enable 会挂)
// 保留判例: 生产库 face_hub.db 与 probe.db 分家; 名字走 sidecar mapping.json;
//          重名/相似脸(0.48)拒绝; del 用 HFFeatureHubFaceRemove (grep 1075)
#include <inspireface.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdarg>
#include <cctype>
#include <cerrno>
#include <unistd.h>       // access()
#include <string>
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#ifndef PROJECT_ROOT
#define PROJECT_ROOT "."   // 兜底: 未注入时退化为当前目录
#endif

// 运行时路径: 由注入的仓库根拼出 (arm=/vsis_video_service, x86=/root/vsis_video_service, 各自编译各自对)
static char kPack[512];
static char kPersonDir[512];
static char kDbDir[512];
static char kHubDb[512];      // HF 接口要 char*, 保持可写数组
static char kPhotoDir[512];
static char kMapping[512];
static void init_paths() {
    snprintf(kPack,      sizeof(kPack),      "%s/third_party/inspireface_sdk/pack/Pikachu", PROJECT_ROOT);
    snprintf(kPersonDir, sizeof(kPersonDir), "%s/person",  PROJECT_ROOT);
    snprintf(kDbDir,     sizeof(kDbDir),     "%s/person/db", PROJECT_ROOT);
    snprintf(kHubDb,     sizeof(kHubDb),     "%s/person/db/face_hub.db", PROJECT_ROOT);
    snprintf(kPhotoDir,  sizeof(kPhotoDir),  "%s/person/photo", PROJECT_ROOT);
    snprintf(kMapping,   sizeof(kMapping),   "%s/person/mapping.json", PROJECT_ROOT);
}

static const float kThreshold = 0.48f;
static HFSession g_session = nullptr;

static void usage() {
    printf("用法:\n"
           "  face_enroll add <名字> <照片>  注册 (照片位于 person/photo/; 恰好一张脸; 重名/相似脸拒绝)\n"
           "  face_enroll list               列出已注册\n"
           "  face_enroll del <名字>         注销 (库+映射+照片一并删除)\n");
}

// 统一退场: 释放 + 资源表, 守卫拦截也走这条路 (家规: 表全 0)
static void die(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
    if (g_session) { HFReleaseInspireFaceSession(g_session); g_session = nullptr; }
    HFFeatureHubDataDisable();
    HFTerminateInspireFace();
    HFDeBugShowResourceStatistics();
    exit(2);
}

// 照片定位: 原样可读则用原样; 否则按 仓库根/路径 再试 (任意 cwd 可调用)
static std::string resolve_photo(const char* p) {
    if (access(p, R_OK) == 0) return std::string(p);
    std::string alt = std::string(PROJECT_ROOT) + "/" + p;
    if (access(alt.c_str(), R_OK) == 0) return alt;
    die("[enroll] 照片不可读: %s (也试过 %s)\n", p, alt.c_str());
    return std::string();
}

// 登记只存 photo/<文件名>; 位置由约定决定, 不存绝对路径 (跨机可移植)
static std::string base_name(const std::string& p) {
    size_t pos = p.find_last_of("/\\");
    return (pos == std::string::npos) ? p : p.substr(pos + 1);
}

static json load_mapping() {
    std::ifstream f(kMapping);
    if (!f.good()) return json::array();            // 首跑无文件 → 空库
    std::stringstream ss; ss << f.rdbuf();
    std::string s = ss.str();
    bool blank = true;
    for (char c : s) if (!isspace((unsigned char)c)) { blank = false; break; }
    if (blank) return json::array();                // 空文件 → 当空库
    try { return json::parse(s); }
    catch (...) { die("[enroll] mapping.json 解析失败, 请人工检查 %s\n", kMapping); }
    return json::array();
}
static void save_mapping(const json& arr) {
    std::ofstream f(kMapping, std::ios::trunc);
    if (!f.good()) die("[enroll] mapping.json 写入失败\n");
    f << arr.dump(2, ' ', false) << std::endl;      // ensure_ascii=false, 中文原样
}

static bool name_ok(const char* s) {
    if (!s || !*s) return false;
    for (const char* p = s; *p; ++p)
        if (*p == '/' || *p == '\\') return false;  // 防路径逃逸
    return true;
}
static int find_by_name(const json& arr, const char* name) {
    for (size_t i = 0; i < arr.size(); ++i)
        if (arr[i].value("name", std::string()) == name) return (int)i;
    return -1;
}
static int find_by_id(const json& arr, long long id) {
    for (size_t i = 0; i < arr.size(); ++i)
        if (arr[i].value("id", -1LL) == id) return (int)i;
    return -1;
}

// probe_persist 同款: Create + ExtractCpy, 所有权全程归我们; 注册照必须恰好一张脸
static HResult extract_feature(HFSession session, const char* img, HFFaceFeature* feat) {
    HFImageBitmap bitmap = nullptr;
    HResult ret = HFCreateImageBitmapFromFilePath(img, 3, &bitmap);
    if (ret != HSUCCEED) { printf("[enroll] bitmap fail: %ld\n", ret); return ret; }
    HFImageStream stream = nullptr;
    ret = HFCreateImageStreamFromImageBitmap(bitmap, HF_CAMERA_ROTATION_0, &stream);
    if (ret != HSUCCEED) {
        printf("[enroll] stream fail: %ld\n", ret);
        HFReleaseImageBitmap(bitmap); return ret;
    }
    HFMultipleFaceData faces = {};
    ret = HFExecuteFaceTrack(session, stream, &faces);
    if (ret != HSUCCEED) {
        printf("[enroll] track fail: %ld\n", ret);
        HFReleaseImageStream(stream); HFReleaseImageBitmap(bitmap); return ret;
    }
    if (faces.detectedNum < 1) {
        printf("[enroll] no face in %s\n", img);
        HFReleaseImageStream(stream); HFReleaseImageBitmap(bitmap); return (HResult)3;
    }
    if (faces.detectedNum > 1) {
        printf("[enroll] %d faces in %s — 注册照必须恰好一张脸\n", faces.detectedNum, img);
        HFReleaseImageStream(stream); HFReleaseImageBitmap(bitmap); return (HResult)3;
    }
    ret = HFCreateFaceFeature(feat);
    if (ret == HSUCCEED) {
        ret = HFFaceFeatureExtractCpy(session, stream, faces.tokens[0], feat->data);
        if (ret != HSUCCEED) HFReleaseFaceFeature(feat);
    }
    HFReleaseImageStream(stream);
    HFReleaseImageBitmap(bitmap);
    return ret;
}

int main(int argc, char** argv) {
    init_paths();
    if (argc < 2) { usage(); return 2; }
    const char* mode = argv[1];
    if (strcmp(mode, "add") != 0 && strcmp(mode, "list") != 0 
     && strcmp(mode, "del") != 0 && strcmp(mode, "query") != 0) {
        printf("未知命令: '%s'\n", mode); usage(); return 2;
    }
    if (strcmp(mode, "add")  == 0 && argc != 4 && argc != 5) { printf("用法: face_enroll add <名字> <照片> [问候语]\n"); return 2; }
    if (strcmp(mode, "list") == 0 && argc != 2) { printf("用法: face_enroll list\n"); return 2; }
    if (strcmp(mode, "del")  == 0 && argc != 3) { printf("用法: face_enroll del <名字>\n"); return 2; }

    // pack 自检: SDK 对缺失包直接 abort(核心转储), 先给人话
    struct stat st;
    if (stat(kPack, &st) != 0) {
        printf("[enroll] pack 不存在: %s\n[enroll] PROJECT_ROOT=%s — 检查 CMake 注入与仓库布局\n",
               kPack, PROJECT_ROOT);
        return 1;
    }
    HResult ret = HFLaunchInspireFace(kPack);
    if (ret != HSUCCEED) { printf("[enroll] launch fail: %ld\n", ret); return 1; }

    mkdir(kPersonDir, 0755);   // 首跑目录保障 (已存在则忽略)
    mkdir(kDbDir, 0755);       // ★ v1.0 漏了这个, 新机首跑 hub enable 会挂
    mkdir(kPhotoDir, 0755);

    HFFeatureHubConfiguration cfg = {};
    cfg.primaryKeyMode = HF_PK_AUTO_INCREMENT;
    cfg.enablePersistence = 1;
    cfg.persistenceDbPath = kHubDb;          // 生产库, 与 probe.db 分家
    cfg.searchMode = HF_SEARCH_MODE_EAGER;
    cfg.searchThreshold = kThreshold;
    ret = HFFeatureHubDataEnable(cfg);
    if (ret != HSUCCEED) {
        printf("[enroll] hub fail: %ld\n", ret);
        HFTerminateInspireFace(); return 1;
    }
    json mapping = load_mapping();

    if (strcmp(mode, "add") == 0) {
        const char* name = argv[2];
        const char* img  = argv[3];
        if (!name_ok(name))
            die("[enroll] 非法名字: '%s' (不得含 / 或 \\)\n", name);
        int idx = find_by_name(mapping, name);
        if (idx >= 0)
            die("[enroll] 重名拒绝: '%s' 已注册 (id=%lld) — 如需更换照片, 先 del 再 add\n",
                name, mapping[idx].value("id", -1LL));
        std::string photoPath = resolve_photo(img);          // 找不到直接人话退场

        HFSessionCustomParameter param = {};
        param.enable_recognition = 1;
        ret = HFCreateInspireFaceSession(param, HF_DETECT_MODE_ALWAYS_DETECT, 1, -1, -1, &g_session);
        if (ret != HSUCCEED) die("[enroll] session fail: %ld\n", ret);
        HFFaceFeature feat = {};
        ret = extract_feature(g_session, photoPath.c_str(), &feat);
        if (ret != HSUCCEED) die("[enroll] extract fail: %ld\n", ret);

        // 相似脸自查: 同一 0.48 线, 防一人两名
        HFloat conf = 0;
        HFFaceFeatureIdentity matched = {};
        ret = HFFeatureHubFaceSearch(feat, &conf, &matched);
        if (ret == HSUCCEED && matched.id != -1) {
            long long mid = (long long)matched.id;
            int mi = find_by_id(mapping, mid);
            std::string who = (mi >= 0) ? mapping[mi].value("name", std::string("?"))
                                        : std::string("(映射缺名)");
            HFReleaseFaceFeature(&feat);
            die("[enroll] 相似脸拒绝: 与已注册 id=%lld (%s) 相似度 %.3f >= %.2f — 疑同一人重复注册\n"
                "[enroll] 若确为不同人: 请换一张差异更大的注册照, 或先处理冲突条目\n",
                mid, who.c_str(), conf, kThreshold);
        }

        std::string photoRel = "photo/" + base_name(photoPath);   // 只登记, 不拷贝
        HFFaceFeatureIdentity ident = {};
        ident.feature = &feat;
        HFaceId allocId = -1;
        ret = HFFeatureHubInsertFeature(ident, &allocId);
        if (ret != HSUCCEED) { HFReleaseFaceFeature(&feat); die("[enroll] insert fail: %ld\n", ret); }
        HFReleaseFaceFeature(&feat);

        json entry = {{"id", (long long)allocId}, {"name", name}, {"photo", photoRel}};
        if (argc >= 5) {
            entry["hello"] = argv[4];
        }else {
            entry["hello"] = std::string(name) + "，您好！";
        }

        mapping.push_back(entry);
        save_mapping(mapping);
        HInt32 count = 0;
        HFFeatureHubGetFaceCount(&count);
        printf("[ENROLL-ADD] id=%lld name=%s photo=%s\n", (long long)allocId, name, photoRel.c_str());
        printf("[ENROLL-ADD] hub count=%d mapping=%d\n", count, (int)mapping.size());

    } else if (strcmp(mode, "list") == 0) {
        HInt32 count = 0;
        HFFeatureHubGetFaceCount(&count);
        printf("[ENROLL-LIST] hub=%d mapping=%d\n", count, (int)mapping.size());
        if (count != (HInt32)mapping.size())
            printf("[ENROLL-LIST] 警告: hub 与 mapping 数目不一致 — 库可能被外部改动\n");
        if (mapping.empty()) printf("  (空库)\n");
        for (size_t i = 0; i < mapping.size(); ++i)
            printf("  id=%lld name=%s photo=%s\n",
                   mapping[i].value("id", -1LL),
                   mapping[i].value("name", std::string("?")).c_str(),
                   mapping[i].value("photo", std::string("?")).c_str());

    } else if (strcmp(mode, "del") == 0) {
        const char* name = argv[2];
        int idx = find_by_name(mapping, name);
        if (idx < 0) die("[enroll] 未找到: '%s'\n", name);
        HFaceId id = (HFaceId)mapping[idx].value("id", -1LL);
        ret = HFFeatureHubFaceRemove(id);           // grep 1075 判例
        if (ret != HSUCCEED) die("[enroll] delete fail: %ld\n", ret);
        std::string photo = mapping[idx].value("photo", std::string());
        mapping.erase(idx);
        save_mapping(mapping);
        // 照片删除: photo 是 person/ 下的相对路径 (photo/<名字>.<ext>)
        // ★ v1.1 bug 修复: 拼接基座用 kPersonDir (= PROJECT_ROOT/person),
        //   原来拼 PROJECT_ROOT/photo/... 少了 person/ 一层 → 永远删不到
        bool shared = false;
        for (size_t i = 0; i < mapping.size(); ++i)
            if (mapping[i].value("photo", std::string()) == photo) shared = true;
        std::string photoAbs = std::string(kPersonDir) + "/" + photo;  // = .../person/photo/...
        if (shared || photo.empty()) {
            printf("[ENROLL-DEL] id=%lld name=%s (照片保留: %s 仍被其他条目引用)\n",
                   (long long)id, name, photo.c_str());
        } else if (remove(photoAbs.c_str()) == 0) {
            printf("[ENROLL-DEL] id=%lld name=%s (照片已删除: %s)\n",
                   (long long)id, name, photo.c_str());
        } else {
            // 失败必须打确切路径 + 系统原因, 不许"不在档"这种含糊话
            printf("[ENROLL-DEL] id=%lld name=%s (照片未删: %s — %s)\n",
                   (long long)id, name, photoAbs.c_str(), strerror(errno));
        }
    } else if (strcmp(mode, "query") == 0) {
        std::string photoPath = resolve_photo(argv[2]);
        HFSessionCustomParameter param = {};
        param.enable_recognition = 1;
        ret = HFCreateInspireFaceSession(param, HF_DETECT_MODE_ALWAYS_DETECT, 1, -1, -1, &g_session);
        if (ret != HSUCCEED) die("[enroll] session fail: %ld\n", ret);
        HFFaceFeature feat = {};
        ret = extract_feature(g_session, photoPath.c_str(), &feat);
        if (ret != HSUCCEED) die("[enroll] extract fail: %ld\n", ret);

        HFloat conf = 0;
        HFFaceFeatureIdentity matched = {};
        ret = HFFeatureHubFaceSearch(feat, &conf, &matched);
        HFReleaseFaceFeature(&feat);
        if (ret != HSUCCEED) die("[enroll] search fail: %ld\n", ret);
        if (matched.id == -1) {
            printf("[QUERY] 未识别 (conf=%.3f < %.2f) — 陌生脸\n", conf, kThreshold);
        } else {
            int mi = find_by_id(mapping, matched.id);
            std::string name = (mi >= 0) ? mapping[mi].value("name", std::string("?"))
                                         : std::string("(映射缺名)");
            std::string hello = (mi >= 0) ? mapping[mi].value("hello", name + "，您好！")
                                          : std::string("您好");     // ★ value() 兜底, 判例已立
            printf("[QUERY] %s (id=%lld conf=%.3f) → %s\n",
                   name.c_str(), (long long)matched.id, conf, hello.c_str());
        }
    }

    if (g_session) { HFReleaseInspireFaceSession(g_session); g_session = nullptr; }
    HFFeatureHubDataDisable();
    HFTerminateInspireFace();
    HFDeBugShowResourceStatistics();
    return 0;
}
