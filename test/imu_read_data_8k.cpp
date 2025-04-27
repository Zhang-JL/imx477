// g++ -o imu_read_data_8k imu_read_data_8k.cpp -lwiringPi

#include <iostream>
#include <unistd.h>         // for usleep
#include <stdint.h>
#include <cstring>          // for memset
#include <csignal>          // for signal handling (SIGINT)
#include <fstream>          // for file operations
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#define SPI_CHANNEL 0       // 使用SPI通道0（CE0）
#define SPI_SPEED 1000000   // SPI速度1MHz，可根据需要调整
#define INT_GPIO_PIN 17     // 连接imu的int1
#define FIFO_GYRO_COUNT 8   // FIFO中陀螺仪数据的数量
#define FIFO_PACKET_SIZE 7  // FIFO中每个数据包的大小（1 byte header + 6 byte gyro数据）

// ICM-42688寄存器地址
#define RESET_DEVICE        0x01
#define REG_BANK_SEL        0x76
#define DEVICE_CONFIG       0x11
#define INT_CONFIG          0x14
#define INT_CONFIG0         0x63
#define INT_CONFIG1         0x64
#define INT_SOURCE0         0x65
#define FIFO_CONFIG         0x16
#define PWR_MGMT0           0x4E
#define GYRO_CONFIG0        0x4F
#define ACCEL_CONFIG0       0x50
#define GYRO_CONFIG1        0x51
#define ACCEL_CONFIG1       0x53
#define FIFO_CONFIG1        0x5F
#define FIFO_COUNTH         0x2E
#define FIFO_COUNTL         0x2F
#define FIFO_DATA           0x30
#define FIFO_CONFIG2        0x60
#define FIFO_CONFIG3        0x61
#define INT_STATUS          0x2D


#define GYRO_CONFIG_STATIC2 0x0B
#define GYRO_CONFIG_STATIC3 0x0C
#define GYRO_CONFIG_STATIC4 0x0D
#define GYRO_CONFIG_STATIC5 0x0E
#define ACCEL_CONFIG_STATIC2 0x03
#define ACCEL_CONFIG_STATIC3 0x04
#define ACCEL_CONFIG_STATIC4 0x05

#define TEMP_DATA1          0x1D
#define TEMP_DATA0          0x1E
#define ACCEL_DATA_X1       0x1F
#define ACCEL_DATA_X0       0x20
#define ACCEL_DATA_Y1       0x21
#define ACCEL_DATA_Y0       0x22
#define ACCEL_DATA_Z1       0x23
#define ACCEL_DATA_Z0       0x24

// 全局变量
int spiFd;
std::vector<int16_t> gyroDataBuffer; // 存储读取的陀螺仪数据
uint64_t lastInterruptTime = 0;
uint64_t currentInterruptTime = 0;
std::mutex dataMutex;
std::condition_variable dataReadyCond;
bool dataReady = false;
volatile bool running = true; // 全局变量，用于控制主循环

// 信号处理函数
void signalHandler(int signum) {
    std::cout << "\n捕获到 Ctrl+C 信号，正在退出程序..." << std::endl;
    running = false; // 设置运行标志为 false，退出主循环
     dataReadyCond.notify_one(); // 唤醒主线程
}

// 获取当前系统时间（微秒）
uint64_t get_current_time_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts); // 使用单调时钟
    return static_cast<uint64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
}

// SPI写寄存器函数
void writeRegister(uint8_t reg, uint8_t data) {
    uint8_t buffer[2];
    buffer[0] = reg & 0x7F;  // 确保最高位为0（写操作）
    buffer[1] = data;
    wiringPiSPIDataRW(SPI_CHANNEL, buffer, 2);
}

// SPI读寄存器函数
void readRegister(uint8_t reg, uint16_t dataLen, uint8_t withHeader, uint8_t* rtnData) {
    uint8_t buffer[dataLen + 1];
    memset(buffer, 0, sizeof(buffer));
    buffer[0] = reg | 0x80;  // 设置最高位为1（读操作）
    wiringPiSPIDataRW(SPI_CHANNEL, buffer, dataLen + 1);
    if (!withHeader) {
        memcpy(rtnData, &buffer[1], dataLen);
    } else {
        memcpy(rtnData, &buffer[0], dataLen + 1);
    }
}

uint8_t spiReadRegister(uint8_t reg) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg | 0x80), 0x00 };
    uint8_t rx[2] = {0};
    wiringPiSPIDataRW(SPI_CHANNEL, tx, 2);
    return tx[1];
}

// 切换寄存器bank
void selectBank(uint8_t bank) {
    writeRegister(REG_BANK_SEL, bank);
}

// 初始化ICM-42688
void initICM42688() {
    // 重置设备
    writeRegister(DEVICE_CONFIG, RESET_DEVICE);
    usleep(100000);  // 延时100ms等待复位完成

    selectBank(0);
    // 传感器相关：
    writeRegister(PWR_MGMT0, 0x0F); // 使能6轴低噪声模式，使能温度传感器
    writeRegister(GYRO_CONFIG0, (0 << 5) | 0x03); // 陀螺仪ODR = 8kHz，量程为±2000dps
    writeRegister(ACCEL_CONFIG0, (1 << 5) | 0x06); // 加速度ODR = 1kHz，设置量程为±8g
    // // 滤波器相关：
    // selectBank(1); // 配置gyro的filter
    // writeRegister(GYRO_CONFIG_STATIC2, 0x01); // bit0=1:关闭NF，bit1=0:使能AAF
    // writeRegister(GYRO_CONFIG_STATIC3, 63 & 0x3F); // bit0~5:GYRO_AAF_DELT
    // uint16_t gyro_aaf_deltsqr = 3968;
    // writeRegister(GYRO_CONFIG_STATIC4, (uint8_t)(gyro_aaf_deltsqr & 0xFF)); // bit0~7:GYRO_AAF_DELTSQR的低八位
    // writeRegister(GYRO_CONFIG_STATIC5, (3 << 4) | (uint8_t)((gyro_aaf_deltsqr >> 8) & 0x0F)); // bit4~7:GYRO_AAF_BITSHIFT
    //                                                                                             // bit0~3:GYRO_AAF_DELTSQR的bit8~11
    // selectBank(2); // 配置加速度计的filter
    // writeRegister(ACCEL_CONFIG_STATIC2, (21 << 1) & 0x7E); // bit1~6:ACCEL_AAF_DELT，bit0=0:使能AAF
    // uint16_t accel_aaf_deltsqr = 440;
    // writeRegister(ACCEL_CONFIG_STATIC3, (uint8_t)(accel_aaf_deltsqr & 0xFF)); // bit0~7:ACCEL_AAF_DELTSQR 低字节
    // writeRegister(ACCEL_CONFIG_STATIC4, (6 << 4) | (uint8_t)((accel_aaf_deltsqr >> 8) & 0x0F)); // bit4~7:ACCEL_AAF_BITSHIFT
    //                                                                                             // bit0~3:ACCEL_AAF_DELTSQR的bit8~11
    selectBank(0);
    // FIFO相关：
    // 启用FIFO数据输出
    writeRegister(FIFO_CONFIG1, (1 << 6) | (1 << 1));  // bit1=1:启用gyro数据存储到FIFO
    // 配置FIFO中断，每8个数据触发一次中断
    // 设置FIFO水位阈值为：7 byte * 8 个 = 56 byte。7 byte 的组成：1 byte header + 6 byte gyro
    writeRegister(FIFO_CONFIG2, FIFO_PACKET_SIZE * FIFO_GYRO_COUNT); // 低byte
    writeRegister(FIFO_CONFIG3, 0x00); // 高byte
    writeRegister(FIFO_CONFIG, (1 << 6)); // FIFO模式：bit6~7=1:Stream模式
    
    // 中断相关：
    writeRegister(INT_CONFIG, 0x03); // bit0=0:int1高电平有效，bit1=1:int1推挽模式，bit2=0:int1脉冲模式(适合高频情况)
    writeRegister(INT_CONFIG1, (1 << 6) | (1 << 5) | (0 << 4));  // bit4=0:确保int引脚生效，bit5=1:短时间的中断解除断言持续时间
                                                                 // bit6=1:短时间的中断脉冲       
    writeRegister(INT_SOURCE0, (1 << 2)); // 启用 FIFO 阈值中断到 INT1
    // 等待配置稳定
    usleep(10000);

    // ========== 回读寄存器检查 ==========
    std::cout << "回读寄存器检查：" << std::endl;

    // Bank 0
    selectBank(0);
    uint8_t pwrMgmt0 = spiReadRegister(PWR_MGMT0);
    uint8_t gyroConfig0 = spiReadRegister(GYRO_CONFIG0);
    uint8_t accelConfig0 = spiReadRegister(ACCEL_CONFIG0);
    std::cout << "PWR_MGMT0: 0x" << std::hex << (int)pwrMgmt0 << std::endl;
    std::cout << "GYRO_CONFIG0: 0x" << std::hex << (int)gyroConfig0 << std::endl;
    std::cout << "ACCEL_CONFIG0: 0x" << std::hex << (int)accelConfig0 << std::endl;

    // Bank 1
    selectBank(1);
    uint8_t gyroStatic2 = spiReadRegister(GYRO_CONFIG_STATIC2);
    uint8_t gyroStatic3 = spiReadRegister(GYRO_CONFIG_STATIC3);
    std::cout << "GYRO_CONFIG_STATIC2: 0x" << std::hex << (int)gyroStatic2 << std::endl;
    std::cout << "GYRO_CONFIG_STATIC3: 0x" << std::hex << (int)gyroStatic3 << std::endl;

    // Bank 2
    selectBank(2);
    uint8_t accelStatic2 = spiReadRegister(ACCEL_CONFIG_STATIC2);
    uint8_t accelStatic3 = spiReadRegister(ACCEL_CONFIG_STATIC3);
    std::cout << "ACCEL_CONFIG_STATIC2: 0x" << std::hex << (int)accelStatic2 << std::endl;
    std::cout << "ACCEL_CONFIG_STATIC3: 0x" << std::hex << (int)accelStatic3 << std::endl;

    // 回到 Bank 0
    selectBank(0);
    uint8_t fifoConfig = spiReadRegister(FIFO_CONFIG);
    uint8_t fifoConfig1 = spiReadRegister(FIFO_CONFIG1);
    uint8_t fifoConfig2 = spiReadRegister(FIFO_CONFIG2);
    uint8_t fifoConfig3 = spiReadRegister(FIFO_CONFIG3);
    uint8_t intConfig = spiReadRegister(INT_CONFIG);
    uint8_t intConfig1 = spiReadRegister(INT_CONFIG1);
    uint8_t intSource0 = spiReadRegister(INT_SOURCE0);

    std::cout << "FIFO_CONFIG: 0x" << std::hex << (int)fifoConfig << std::endl;
    std::cout << "FIFO_CONFIG1: 0x" << std::hex << (int)fifoConfig1 << std::endl;
    std::cout << "FIFO_CONFIG2: 0x" << std::hex << (int)fifoConfig2 << std::endl;
    std::cout << "FIFO_CONFIG3: 0x" << std::hex << (int)fifoConfig3 << std::endl;
    std::cout << "INT_CONFIG: 0x" << std::hex << (int)intConfig << std::endl;
    std::cout << "INT_CONFIG1: 0x" << std::hex << (int)intConfig1 << std::endl;
    std::cout << "INT_SOURCE0: 0x" << std::hex << (int)intSource0 << std::endl;

    // 检查 FIFO 数据计数
    uint8_t fifoCountH = spiReadRegister(FIFO_COUNTH);
    uint8_t fifoCountL = spiReadRegister(FIFO_COUNTL);
    uint16_t fifoCount = (fifoCountH << 8) | fifoCountL;
    std::cout << "FIFO_COUNT: " << fifoCount << " bytes" << std::endl;

    // 检查中断状态
    uint8_t intStatus = spiReadRegister(INT_STATUS);
    std::cout << "INT_STATUS: 0x" << std::hex << (int)intStatus << std::endl;
}

void initIMU() {
    // 设备复位
    selectBank(0);
    writeRegister(DEVICE_CONFIG, 0x01);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    selectBank(0);
    // 关闭待机，低噪模式启用陀螺仪和加速度计，温度传感器启用
    writeRegister(PWR_MGMT0, 0x0F);

    // 设陀螺仪和加速度计 ODR均为1kHz，且量程为陀螺仪±2000dps，加速度计±8g
    writeRegister(GYRO_CONFIG0, (0 << 5) | 0x06); // Gyro ODR=1kHz, FS=±2000dps
    writeRegister(ACCEL_CONFIG0, (1 << 5) | 0x06); // Accel ODR=1kHz, FS=±8g

    // 配置陀螺仪抗混叠滤波器 （AAF）带宽为997Hz，关闭Notch滤波器
    selectBank(1);
    uint8_t val = spiReadRegister(0x0B);
    val = (val | 0x01) & ~0x02; // Bit0置1禁用Notch, Bit1清0启用AAF
    writeRegister(0x0B, val);
    writeRegister(0x0C, 21 & 0x3F); // GYRO_AAF_DELT=21 (对应997Hz)
    uint16_t aaf_deltsqr = 440;
    writeRegister(0x0D, aaf_deltsqr & 0xFF);
    uint8_t tmp = spiReadRegister(0x0E);
    tmp = (tmp & 0x0F) | (6 << 4); // GYRO_AAF_BITSHIFT=6
    tmp = (tmp & 0xF0) | ((aaf_deltsqr >> 8) & 0x0F);
    writeRegister(0x0E, tmp);

    // 加速度计AAF配置同陀螺仪，997Hz，启用AAF
    selectBank(2);
    uint8_t val_acc = spiReadRegister(0x04);
    val_acc = (val_acc & 0x81) | ((21 <<1) & 0x7E); // ACCEL_AAF_DELT=21 设置，启用AAF(bit0=0)
    writeRegister(0x04, val_acc);
    uint16_t acc_aaf_deltsqr = 440;
    writeRegister(0x05, acc_aaf_deltsqr & 0xFF);
    tmp = spiReadRegister(0x06);
    tmp = (tmp & 0x0F) | (6 << 4);
    tmp = (tmp & 0xF0) | ((acc_aaf_deltsqr >> 8) & 0x0F);
    writeRegister(0x06, tmp);

    // 回到Bank0
    selectBank(0);
    // 配置FIFO为流模式，但本示例不使用FIFO读取数据，读寄存器实时采样
    writeRegister(FIFO_CONFIG, 0x00);   // 关闭FIFO，避免数据混乱
    // 配置中断（不配置中断，采用定时读取，INT_GPIO不用）
    // 中断配置可选，故省略

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

// 从FIFO中读取数据
void readFIFO() {
    //// std::lock_guard<std::mutex> lock(dataMutex);
    selectBank(0);
    // 读取FIFO计数
    uint8_t countH, countL;
    readRegister(FIFO_COUNTH, 1, FALSE, &countH); // FIFO计数高字节
    readRegister(FIFO_COUNTL, 1, FALSE, &countL); // FIFO计数低字节
    uint16_t count = (countH << 8) | countL;
    if (count < FIFO_PACKET_SIZE * FIFO_GYRO_COUNT) {
        std::cerr << "FIFO数据不足，当前计数: " << count << std::endl;
    }
    // 解析FIFO数据 todo: 如果FIFO读取较慢，可能会多读一组数据，需要特殊处理
    int16_t idx = 0;
    uint8_t* fifoDataPtr;
    readRegister(FIFO_DATA, count, FALSE, fifoDataPtr);
    gyroDataBuffer.clear();
    while (idx + FIFO_PACKET_SIZE <= count) {
        uint8_t header = fifoDataPtr[idx++];
        if (header & 0x20) {
            // 包含陀螺仪数据，解析6字节
            if (idx + 6 > count) break; // 数据长度不够，退出
            int16_t gx = (int16_t)((fifoDataPtr[idx] << 8) | fifoDataPtr[idx + 1]);
            int16_t gy = (int16_t)((fifoDataPtr[idx + 2] << 8) | fifoDataPtr[idx + 3]);
            int16_t gz = (int16_t)((fifoDataPtr[idx + 4] << 8) | fifoDataPtr[idx + 5]);
            idx += 6;
            // 将陀螺仪数据存入缓冲区
            gyroDataBuffer.push_back(gx);
            gyroDataBuffer.push_back(gy);
            gyroDataBuffer.push_back(gz);
        } else {
            // std::cerr << "FIFO数据包头错误，跳过数据包" << std::endl;
        }
    }
}

// 中断处理函数
void interruptHandler() {
    //// std::unique_lock<std::mutex> lock(dataMutex);
    // 记录当前中断发生的系统时间
    lastInterruptTime = currentInterruptTime;
    currentInterruptTime = get_current_time_us();
    // 清除中断状态
    uint8_t intStatus;
    readRegister(INT_STATUS, 1, FALSE, &intStatus);
    if (intStatus & 0x04) { // 检查FIFO水印中断，bit2=1:FIFO_THS_INT置1说明fifo数据达到阈值
        dataReady = true;
         dataReadyCond.notify_one();
    }
}

int main() {
    // 注册信号处理函数
    signal(SIGINT, signalHandler);
    // 初始化wiringPi
    if (wiringPiSetupGpio() < 0) {
        std::cerr << "Unable to setup wiringPi\n";
        return 1;
    }
    // 初始化SPI
    spiFd = wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED);
    if (spiFd < 0) {
        std::cerr << "Unable to setup SPI\n";
        return 1;
    }
    // 打开数据文件
    std::ofstream dataFile("imu_data.txt");
    if (!dataFile.is_open()) {
        std::cerr << "无法打开数据文件！" << std::endl;
        return -1;
    }
 
    // 初始化ICM-42688
    initICM42688();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 设置GPIO中断引脚
    pinMode(INT_GPIO_PIN, INPUT);
    pullUpDnControl(INT_GPIO_PIN, PUD_DOWN);
    if (wiringPiISR(INT_GPIO_PIN, INT_EDGE_RISING, &interruptHandler) < 0) {
        std::cerr << "Failed to setup interrupt!" << std::endl;
        return -1;
    }

    // 在初始化后添加WHO_AM_I检查
    uint8_t whoami2;
    readRegister(0x75, 1, FALSE, &whoami2);
    std::cerr << "WHO_AM_I: " << std::hex << (int)whoami2 << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // // 检查 FIFO 数据计数
    // uint8_t fifoCountH = spiReadRegister(FIFO_COUNTH);
    // uint8_t fifoCountL = spiReadRegister(FIFO_COUNTL);
    // uint16_t fifoCount = (fifoCountH << 8) | fifoCountL;
    // std::cout << "FIFO_COUNT: " << fifoCount << " bytes" << std::endl;
    // readFIFO();  // 将实际读取操作移到主循环
    // dataReady = false;
    // // 检查 FIFO 数据计数
    // fifoCountH = spiReadRegister(FIFO_COUNTH);
    // fifoCountL = spiReadRegister(FIFO_COUNTL);
    // fifoCount = (fifoCountH << 8) | fifoCountL;
    // std::cout << "FIFO_COUNT: " << fifoCount << " bytes" << std::endl;
    writeRegister(0x4B, 0x02);
    // 主循环
    while (running) {
        // std::cout << "dgb:123" << std::endl;
        // 等待中断处理程序读取数据完成
        //// std::unique_lock<std::mutex> lock(dataMutex);
        //// dataReadyCond.wait(lock, [] { return dataReady || !running; });
        if (!running) break;
        if (dataReady) {
            std::cout << "dgb:Interrupt occurred!" << std::endl;
            readFIFO();  // 将实际读取操作移到主循环
            dataReady = false;
        }
        // 判断数据长度是否合格
        uint16_t fifoSmpCnt = gyroDataBuffer.size() / 3;
        if (fifoSmpCnt > FIFO_GYRO_COUNT){
            std::cerr << "FIFO数据包采样个数异常，CNT为: " << fifoSmpCnt << std::endl;
        }
        // 生成陀螺仪数据的时间戳
        uint64_t deltaTime = currentInterruptTime - lastInterruptTime;
        uint64_t timeStep = (uint64_t)((double)deltaTime / (double)fifoSmpCnt);
        std::vector<uint64_t> gyroTimestamps;
        for (uint16_t i = 0; i < fifoSmpCnt; ++i) {
            uint64_t timestamp = currentInterruptTime - (FIFO_GYRO_COUNT * fifoSmpCnt - i - 1) * timeStep;
            gyroTimestamps.push_back(timestamp);
        }
        // 将数据写入文件
        uint16_t gyroDataIndex = 0;
        for (uint16_t i = 0; i < fifoSmpCnt; ++i) {
            dataFile << gyroTimestamps[i] << "\t";
            dataFile << gyroDataBuffer[gyroDataIndex++] << "\t";
            dataFile << gyroDataBuffer[gyroDataIndex++] << "\t";
            dataFile << gyroDataBuffer[gyroDataIndex++];
            dataFile << std::endl;
        }
        gyroDataBuffer.clear();
        std::this_thread::sleep_for(std::chrono::microseconds(500)); // 延时
    }
    // 清理资源
    dataFile.close(); // 关闭文件
    std::cout << "程序已退出，文件已关闭。" << std::endl;
    return 0;
}