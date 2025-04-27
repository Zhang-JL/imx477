// g++ -o video_recorder video_recorder.cpp -lgpiod -pthread

#include <iostream>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>
#include <csignal>
#include <sys/wait.h>
#include <fcntl.h>
#include <atomic>
#include <thread>
#include <gpiod.h>
#include <fstream>
#include <ctime>
#include <sys/time.h>

// 配置宏（编译时通过-DDEBUG或-DDEBUG2启用）
#define DEBUG
// #define DEBUG2

// ========== 配置宏 ==========
#define WIDTH                   2028
#define HEIGHT                  1080
#define FPS                     30
#define FRAME_COUNT             200
#define SHUTTER_US              10000
#define GPIO_CHIP               "/dev/gpiochip0"
#define GPIO_PIN                18
#define STABILIZATION_TIME      1

#ifdef DEBUG
#define LOG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define LOG(msg)
#endif

// ========== 全局状态 ==========
std::atomic<bool> running(true);
std::atomic<int> interrupt_counter(0);
pid_t recording_pid = -1;
struct termios orig_termios;

// ========== 时间记录函数 ==========
uint64_t get_current_time_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts); // 使用单调时钟
    return static_cast<uint64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
}

void log_interrupt_time(uint64_t cnt, uint64_t time_us) {
    std::ofstream log_file("interrupt_log.txt", std::ios::app); // 以追加模式打开文件
    if (log_file.is_open()) {
        log_file << cnt << " " << time_us << std::endl;
        log_file.close();
    } else {
        std::cerr << "无法打开日志文件" << std::endl;
    }
}

void initialize_log_file() {
    std::ofstream log_file("interrupt_log.txt", std::ios::trunc); // 以截断模式打开文件
    if (log_file.is_open()) {
        log_file << "中断日志记录开始" << std::endl;
        log_file.close();
    } else {
        std::cerr << "无法初始化日志文件" << std::endl;
    }
}

// ========== 终端设置 ==========
void disableRawMode() {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

void enableRawMode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disableRawMode);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

// ========== 录制函数 ==========
void start_recording() {
    if (recording_pid != -1) return;

    // 清零中断计数器
    interrupt_counter = 0;

    pid_t pid = fork();
    if (pid == 0) {
        // 重定向输出控制
        #ifndef DEBUG
        int dev_null = open("/dev/null", O_WRONLY);
        dup2(dev_null, STDOUT_FILENO);
        dup2(dev_null, STDERR_FILENO);
        close(dev_null);
        #endif

        execlp("libcamera-vid", "libcamera-vid",
               "--frames", std::to_string(FRAME_COUNT).c_str(),
               "--width", std::to_string(WIDTH).c_str(),
               "--height", std::to_string(HEIGHT).c_str(),
               "--framerate", std::to_string(FPS).c_str(),
               "--shutter", std::to_string(SHUTTER_US).c_str(),
               "--buffer-count", std::to_string(12).c_str(),
               "--codec", "h264",
               "-o", "raw_video.h264",
               "--nopreview",
               "--flush",
               NULL);
        exit(EXIT_FAILURE);
        sleep(STABILIZATION_TIME);
    } else if (pid > 0) {
        recording_pid = pid;
        std::cout << "开始录制..." << std::endl;
    }
}

void stop_recording() {
    if (recording_pid == -1) return;
    kill(recording_pid, SIGINT);
    waitpid(recording_pid, NULL, 0);
    recording_pid = -1;

    std::cout << "\n停止录制，开始格式转换..." << std::endl;
    std::string cmd = "ffmpeg -y -i raw_video.h264 -c copy output.mp4";
    if (system(cmd.c_str()) != 0) {
        std::cerr << "格式转换失败！" << std::endl;
    } else {
        std::cout << "视频已保存为：output.mp4" << std::endl;
    }
    remove("raw_video.h264");
}

// ========== 中断监听线程 ==========
void gpio_interrupt_thread() {
    struct gpiod_chip* chip = gpiod_chip_open(GPIO_CHIP);
    if (!chip) {
        std::cerr << "无法打开 GPIO 芯片" << std::endl;
        return;
    }

    struct gpiod_line* line = gpiod_chip_get_line(chip, GPIO_PIN);
    if (!line) {
        std::cerr << "无法获取 GPIO 引脚" << std::endl;
        gpiod_chip_close(chip);
        return;
    }

    if (gpiod_line_request_falling_edge_events(line, "interrupt_monitor") < 0) {
        std::cerr << "无法设置下降沿中断" << std::endl;
        gpiod_chip_close(chip);
        return;
    }

    struct timespec timeout = {0, 50000000};  // 50ms 轮询

    while (running.load()) {
        int ret = gpiod_line_event_wait(line, &timeout);
        if (ret == 1) {
            struct gpiod_line_event event;
            if (gpiod_line_event_read(line, &event) == 0) {
                interrupt_counter++;
                uint64_t time_us = get_current_time_us(); // 获取当前时间
                log_interrupt_time(interrupt_counter, time_us); // 记录时间到文件
                std::cout << "\r中断计数: " << interrupt_counter.load()
                          << " 时间: " << time_us << " us" << std::flush;
            }
        }
    }

    gpiod_line_release(line);
    gpiod_chip_close(chip);
}

// ========== 主程序入口 ==========
int main() {
    initialize_log_file();
    enableRawMode();

    // 非阻塞标准输入
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    std::thread gpio_thread(gpio_interrupt_thread);

    std::cout << "操作指南：\n"
              << "  A - 开始录制\n"
              << "  Q - 退出程序\n";

    while (running.load()) {
        char c;
        if (read(STDIN_FILENO, &c, 1) == 1) {
            switch (tolower(c)) {
                case 'a':
                    start_recording();
                    break;
                case 'q':
                    running = false;
                    stop_recording();
                    break;
            }
        }
        usleep(10000);
    }

    gpio_thread.join();
    std::cout << "\n退出成功。" << std::endl;
    return 0;
}