// imu_camera_sync_recorder.cpp
// 编译指令：g++ -o two_sensor_data two_sensor_data.cpp -lwiringPi -lpthread

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <cstdint>
#include <cstring>
#include <csignal>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <sys/wait.h>

// SPI 配置
#define SPI_CHANNEL 0
#define SPI_SPEED 10000000 // 10MHz SPI
#define READ_LEN 14        // 2 temp + 6 accel + 6 gyro bytes

// ICM-42688 寄存器地址
#define REG_BANK_SEL        0x76
#define DEVICE_CONFIG       0x11
#define PWR_MGMT0           0x4E
#define GYRO_CONFIG0        0x4F
#define ACCEL_CONFIG0       0x50
#define FIFO_CONFIG         0x16
#define INT_STATUS          0x2D
#define WHO_AM_I            0x75
#define REG_TEMP_DATA1      0x1D

// 全局变量
volatile std::atomic<bool> running(true);
pid_t video_pid = -1;
std::mutex mtx_start;
std::condition_variable cv_start;
bool imu_started = false;
uint64_t start_timestamp = 0;

// 数据结构
struct IMUData {
    uint64_t timestamp_us;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
};

std::vector<IMUData> imuDataBuffer;
std::vector<uint64_t> vsyncTimestamps;
std::mutex bufferMutex;

// GPIO 配置
#define VSYNC_GPIO_PIN 18 // 根据实际连接的 GPIO 引脚修改

// 信号处理函数，捕获 Ctrl+C
void signalHandler(int signum) {
    running.store(false);

    // 停止视频录制
    if (video_pid > 0) {
        kill(video_pid, SIGINT);
        waitpid(video_pid, NULL, 0);
        video_pid = -1;
    }
}

// SPI 通信函数
bool spiInit() {
    if (wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) < 0) {
        std::cerr << "SPI setup failed!" << std::endl;
        return false;
    }
    return true;
}

uint8_t spiReadRegister(uint8_t reg) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg | 0x80), 0x00 };
    wiringPiSPIDataRW(SPI_CHANNEL, tx, 2);
    return tx[1];
}

void spiWriteRegister(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7F), value };
    wiringPiSPIDataRW(SPI_CHANNEL, tx, 2);
}

bool spiReadBytes(uint8_t reg, uint8_t* buffer, int length) {
    if (length > 255) return false;
    uint8_t tx[length + 1];
    memset(tx, 0, sizeof(tx));
    tx[0] = reg | 0x80;
    wiringPiSPIDataRW(SPI_CHANNEL, tx, length + 1);
    memcpy(buffer, &tx[1], length);
    return true;
}

void selectBank(uint8_t bank) {
    spiWriteRegister(REG_BANK_SEL, bank);
}

void initIMU() {
    selectBank(0);
    spiWriteRegister(DEVICE_CONFIG, 0x01); // 重置设备
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    spiWriteRegister(PWR_MGMT0, 0x0F); // 启用陀螺仪和加速度计，低噪声模式

    spiWriteRegister(GYRO_CONFIG0, 0x06);  // 陀螺仪 ±2000dps, ODR=1kHz
    spiWriteRegister(ACCEL_CONFIG0, 0x26); // 加速度计 ±8g, ODR=1kHz

    spiWriteRegister(FIFO_CONFIG, 0x00); // 关闭 FIFO

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

int16_t combineBytes(uint8_t highByte, uint8_t lowByte) {
    return static_cast<int16_t>((highByte << 8) | lowByte);
}

// 获取当前时间戳（微秒）
uint64_t get_current_timestamp_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
}

// IMU 数据采集线程
void imu_thread_function() {
    // 等待开始信号
    {
        std::unique_lock<std::mutex> lock(mtx_start);
        cv_start.wait(lock, []{ return imu_started; });
    }

    // 初始化 IMU
    if (!spiInit()) {
        std::cerr << "SPI initialization failed!" << std::endl;
        running.store(false);
        return;
    }
    initIMU();

    // 检查 WHO_AM_I
    uint8_t whoami = spiReadRegister(WHO_AM_I);
    if (whoami != 0x47) {
        std::cerr << "WHO_AM_I mismatch: 0x" << std::hex << static_cast<int>(whoami) << std::endl;
        running.store(false);
        return;
    } else {
        std::cout << "WHO_AM_I: 0x47" << std::endl;
    }

    uint8_t tempBuf[2];
    uint8_t accelBuf[6];
    uint8_t gyroBuf[6];
    auto nextTime = std::chrono::steady_clock::now();

    while (running.load()) {
        uint8_t intStatus = spiReadRegister(INT_STATUS);
        if (intStatus & (1<<3)) {
            nextTime += std::chrono::microseconds(900); // 1kHz 采样

            uint64_t ts = get_current_timestamp_us() - start_timestamp;

            // 读取温度数据
            if (!spiReadBytes(REG_TEMP_DATA1, tempBuf, 2)) {
                std::cerr << "Failed to read temperature data" << std::endl;
                continue;
            }
    
            // 读取加速度计数据
            spiReadBytes(0x1F, &accelBuf[0], 1);
            spiReadBytes(0x20, &accelBuf[1], 1);
            spiReadBytes(0x21, &accelBuf[2], 1);
            spiReadBytes(0x22, &accelBuf[3], 1);
            spiReadBytes(0x23, &accelBuf[4], 1);
            spiReadBytes(0x24, &accelBuf[5], 1);

            // 读取陀螺仪数据
            spiReadBytes(0x25, &gyroBuf[0], 1);
            spiReadBytes(0x26, &gyroBuf[1], 1);
            spiReadBytes(0x27, &gyroBuf[2], 1);
            spiReadBytes(0x28, &gyroBuf[3], 1);
            spiReadBytes(0x29, &gyroBuf[4], 1);
            spiReadBytes(0x2A, &gyroBuf[5], 1);
    
            // 解析数据
            int16_t temp_raw = combineBytes(tempBuf[0], tempBuf[1]);
            int16_t ax = combineBytes(accelBuf[0], accelBuf[1]);
            int16_t ay = combineBytes(accelBuf[2], accelBuf[3]);
            int16_t az = combineBytes(accelBuf[4], accelBuf[5]);
            int16_t gx = combineBytes(gyroBuf[0], gyroBuf[1]);
            int16_t gy = combineBytes(gyroBuf[2], gyroBuf[3]);
            int16_t gz = combineBytes(gyroBuf[4], gyroBuf[5]);

            IMUData imuData = { ts, ax, ay, az, gx, gy, gz };

            // 将数据加入缓冲区
            {
                std::lock_guard<std::mutex> lock(bufferMutex);
                imuDataBuffer.push_back(imuData);
            }

            std::this_thread::sleep_until(nextTime);
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(3));
            continue;
        }
    }
}

// VSYNC 中断处理函数
void vsync_interrupt_handler(void) {
    uint64_t ts = get_current_timestamp_us() - start_timestamp;

    // 将时间戳加入缓冲区
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        vsyncTimestamps.push_back(ts);
    }
}

// VSYNC 中断监听线程
void vsync_thread_function() {
    // 初始化 GPIO
    if (wiringPiSetupGpio() < 0) {
        std::cerr << "Failed to setup WiringPi GPIO!" << std::endl;
        running.store(false);
        return;
    }

    // 设置 GPIO 引脚为输入模式
    pinMode(VSYNC_GPIO_PIN, INPUT);
    // 根据需要设置上拉/下拉电阻
    pullUpDnControl(VSYNC_GPIO_PIN, PUD_UP);

    // 监听中断
    if (wiringPiISR(VSYNC_GPIO_PIN, INT_EDGE_FALLING, &vsync_interrupt_handler) < 0) {
        std::cerr << "Failed to setup VSYNC interrupt!" << std::endl;
        running.store(false);
        return;
    }

    // 等待运行结束
    while (running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 视频录制函数
void start_video_recording() {
    video_pid = fork();
    if (video_pid == 0) {
        // 子进程，执行视频录制命令
        execlp("libcamera-vid", "libcamera-vid",
               "--width", "1920",
               "--height", "1080",
               "--framerate", "30",
               "--codec", "h264",
               "-o", "raw_video.h264",
               "--timeout", "0", // 无限时长，直到被中断
               "--nopreview",
               NULL);
        exit(EXIT_FAILURE);
    } else if (video_pid > 0) {
        std::cout << "开始视频录制..." << std::endl;
    } else {
        std::cerr << "无法启动视频录制进程！" << std::endl;
    }
}

// 主函数
int main() {
    signal(SIGINT, signalHandler);

    // 在程序开始时清空日志文件
    {
        std::ofstream imuFile("imu_data.txt", std::ios::trunc);
        if (imuFile.is_open()) {
            imuFile << "Timestamp_us\tAccelX\tAccelY\tAccelZ\tGyroX\tGyroY\tGyroZ\n";
            imuFile.close();
        } else {
            std::cerr << "无法初始化 imu_data.txt 文件！" << std::endl;
        }

        std::ofstream vsyncFile("vsync_timestamps.txt", std::ios::trunc);
        if (vsyncFile.is_open()) {
            vsyncFile << "Timestamp_us\n";
            vsyncFile.close();
        } else {
            std::cerr << "无法初始化 vsync_timestamps.txt 文件！" << std::endl;
        }
    }

    // 获取开始时间戳
    start_timestamp = get_current_timestamp_us();

    // 启动 IMU 采集线程
    std::thread imuThread(imu_thread_function);

    // 启动 VSYNC 中断监听线程
    std::thread vsyncThread(vsync_thread_function);

    // 通知线程开始
    {
        std::lock_guard<std::mutex> lock(mtx_start);
        imu_started = true;
    }
    cv_start.notify_all();

    // 启动视频录制
    start_video_recording();

    // 等待线程结束
    imuThread.join();
    vsyncThread.join();

    // 停止视频录制
    if (video_pid > 0) {
        kill(video_pid, SIGINT);
        waitpid(video_pid, NULL, 0);
        video_pid = -1;
    }

    // 将缓冲区数据追加保存到文件
    {
        std::lock_guard<std::mutex> lock(bufferMutex);

        // 追加保存 IMU 数据
        std::ofstream imuFile("imu_data.txt", std::ios::app);
        if (imuFile.is_open()) {
            for (const auto& data : imuDataBuffer) {
                imuFile << data.timestamp_us << '\t'
                        << data.ax << '\t' << data.ay << '\t' << data.az << '\t'
                        << data.gx << '\t' << data.gy << '\t' << data.gz << '\n';
            }
            imuFile.close();
            std::cout << "IMU 数据已保存到 imu_data.txt" << std::endl;
        } else {
            std::cerr << "无法打开 imu_data.txt 文件！" << std::endl;
        }

        // 追加保存 VSYNC 时间戳
        std::ofstream vsyncFile("vsync_timestamps.txt", std::ios::app);
        if (vsyncFile.is_open()) {
            for (const auto& ts : vsyncTimestamps) {
                vsyncFile << ts << '\n';
            }
            vsyncFile.close();
            std::cout << "VSYNC 时间戳已保存到 vsync_timestamps.txt" << std::endl;
        } else {
            std::cerr << "无法打开 vsync_timestamps.txt 文件！" << std::endl;
        }
    }

    // 视频转换为 mp4 格式
    std::cout << "正在转换视频格式为 mp4..." << std::endl;
    int ret = system("ffmpeg -y -i raw_video.h264 -c copy output.mp4");
    if (ret == 0) {
        std::cout << "视频已保存为 output.mp4" << std::endl;
        remove("raw_video.h264");
    } else {
        std::cerr << "视频格式转换失败！" << std::endl;
    }

    return 0;
}