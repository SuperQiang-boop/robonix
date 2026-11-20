#include <unistd.h>
#include <libgen.h>
#include <limits.h>

#include <string>
#include <vector>
#include <iostream>
#include <filesystem>

#include "VideoServiceAuxer.h"
#include "VideoDef.h"
#include "yaml-cpp/yaml.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"   // 彩色控制台输出
#include "spdlog/sinks/rotating_file_sink.h"   // 循环滚动文件输出

namespace Vsis {

void VideoServiceAuxer::init() {
    initAppDir();   // 一切路径的基础, 必须最先
    readConfig();   // 填充 m_logMode / m_logLevel
    setLogger();    // 之后才允许使用 SPDLOG_*
}

void VideoServiceAuxer::initAppDir() {
    char path[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", path, PATH_MAX - 1);
    if (count == -1) {
        std::cerr << "[Auxer] readlink /proc/self/exe failed" << std::endl;
        exit(exitcode::FAIL_CONFIG);
    }
    path[count] = '\0';
    m_appDir = std::string(dirname(path)) + "/../";   // bin/ 的上一级 = 工程根
}

void VideoServiceAuxer::readConfig() {
    const std::string ymlFile = getDir("config") + "cfg.yaml";
    try {
        YAML::Node config = YAML::LoadFile(ymlFile);
        m_logMode   = config["log"]["mode"].as<int>(0);    // 键缺省时用默认值
        m_logLevel  = config["log"]["level"].as<int>(1);

        m_videoSource       = config["video"]["source"].as<std::string>("");
        m_videoReconnectMs  = config["video"]["reconnect_interval_ms"].as<int>(3000);
        m_videoWidth        = config["video"]["width"].as<int>(1280);
        m_videoHeight       = config["video"]["height"].as<int>(720);
        
        m_searchThreshold   = config["face"]["search_threshold"].as<double>(0.48);
        m_greetCooldownSec  = config["face"]["greet_cooldown_sec"].as<int>(30);
        m_maxFaces          = config["face"]["max_faces"].as<int>(5);
        m_confirmHits       = config["face"]["confirm_hits"].as<int>(3);
        m_absentSec         = config["face"]["absent_sec"].as<int>(3);

        m_audioHost = config["audio"]["host"].as<std::string>("127.0.0.1");
        m_audioPort = config["audio"]["port"].as<int>(10086);

    } catch (const std::exception& e) {
        // 此刻日志系统尚未装配, 只能 stderr
        std::cerr << "[Auxer] 读取配置失败: " << ymlFile << " - " << e.what() << std::endl;
        exit(exitcode::FAIL_CONFIG);
    }
}

void VideoServiceAuxer::setLogger() {
    if ((m_logMode < 0) || (m_logMode > 2)) {
        std::cerr << "[Auxer] Invalid log mode value = " << m_logMode << std::endl;
        exit(exitcode::FAIL_CONFIG);
    }
    if ((m_logLevel < 0) || (m_logLevel > 5)) {
        std::cerr << "[Auxer] Invalid log level value = " << m_logLevel << std::endl;
        exit(exitcode::FAIL_CONFIG);
    }

    try {
        // 文件日志需要目录先存在
        if (0 != m_logMode) {
            std::filesystem::create_directories(m_appDir + "logs/");
        }

        std::vector<spdlog::sink_ptr> sinks;
        if (1 != m_logMode) {   // 控制台槽
            sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
        }
        if (0 != m_logMode) {   // 文件槽: 单文件 5MB, 保留 3 个备份
            sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                m_appDir + "logs/robot.log", 1024 * 1024 * 5, 3));
        }

        // 注册为全局默认日志器
        auto logger = std::make_shared<spdlog::logger>("robot_logger", sinks.begin(), sinks.end());
        logger->flush_on(spdlog::level::info);
        spdlog::set_default_logger(logger);

        // 全局日志级别
        switch (m_logLevel) {
            case 0: spdlog::set_level(spdlog::level::trace);    break;
            case 1: spdlog::set_level(spdlog::level::debug);    break;
            case 2: spdlog::set_level(spdlog::level::info);     break;
            case 3: spdlog::set_level(spdlog::level::warn);     break;
            case 4: spdlog::set_level(spdlog::level::err);      break;
            default: spdlog::set_level(spdlog::level::critical); break;
        }

        // debug/trace 带线程号, info 以上精简
        if (m_logLevel < 2)
            spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [Thread%t] %v");
        else
            spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    } catch (const spdlog::spdlog_ex& ex) {
        std::cerr << "[Auxer] 日志系统初始化失败: " << ex.what() << std::endl;
        exit(exitcode::FAIL_CONFIG);
    }

    SPDLOG_DEBUG("[Auxer] log_mode  = {}", m_logMode);
    SPDLOG_DEBUG("[Auxer] log_level = {}", m_logLevel);
}


} // namespace Vsis
