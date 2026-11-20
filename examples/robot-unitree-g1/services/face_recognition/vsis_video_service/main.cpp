#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <cstdlib> 

#include "VideoServiceManager.h"
#include "spdlog/spdlog.h"
#include <opencv2/core/utils/logger.hpp>


using namespace std::chrono_literals;

std::atomic<bool> g_shutdown_requested{false};
// exchange 用在信号处理函数里, 前提是无锁
static_assert(std::atomic<bool>::is_always_lock_free, "signal handler requires lock-free atomic");

void signal_handler(int sig) {
    if (g_shutdown_requested.exchange(true)) {
        _exit(128 + sig);   // 第二次信号: 异步信号安全地立即退出
    }
}

int main() {
    // opencv日志
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
    
    // 1. 注册信号
    struct sigaction sa{};
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    if (sigaction(SIGINT, &sa, nullptr) < 0 || sigaction(SIGTERM, &sa, nullptr) < 0) {
        std::cerr << "[main] 信号注册失败: " << strerror(errno) << std::endl;
        return EXIT_FAILURE;
    }

    // SIGPIPE: 对端断开时让 write 返回 EPIPE 错误, 而不是杀进程
    sa.sa_handler = SIG_IGN;
    if (sigaction(SIGPIPE, &sa, nullptr) < 0) {
        std::cerr << "[main] SIGPIPE 忽略失败: " << strerror(errno) << std::endl;
        return EXIT_FAILURE;
    }

    // 2. 大管家
    auto& mngr = Vsis::VideoServiceManager::Instance;
    mngr.init();
    mngr.start();

    SPDLOG_INFO("系统已就绪, 等待退出信号 (Ctrl+C)...");
    while (!g_shutdown_requested) {
        std::this_thread::sleep_for(100ms); 
    }

    SPDLOG_INFO("收到退出信号, 正在关闭服务");
    mngr.stop();
    SPDLOG_INFO("系统已安全退出");
    return EXIT_SUCCESS;
}
