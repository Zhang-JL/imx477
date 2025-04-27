// g++ -o imu_read_data_1k imu_read_data_1k.cpp -lwiringPi -pthread

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <cstdint>
#include <cstring>
#include <csignal>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#define SPI_CHANNEL 0
#define SPI_SPEED 1000000 // 1MHz SPI
#define READ_LEN 14       // 6 gyro + 6 accel + 2 temp bytes

// ICM-42688寄存器地址
#define REG_BANK_SEL        0x76
#define DEVICE_CONFIG       0x11
#define PWR_MGMT0           0x4E
#define GYRO_CONFIG0        0x4F
#define ACCEL_CONFIG0       0x50
#define FIFO_CONFIG         0x16
#define FIFO_CONFIG1        0x5F
#define INT_CONFIG          0x14
#define INT_CONFIG1         0x64
#define INT_SOURCE0         0x65
#define INT_STATUS          0x2D

// 数据寄存器起始地址，重复读取14字节:
#define REG_TEMP_DATA1      0x1D
#define REG_ACC_DATA_X1     0x1F
#define REG_GYRO_DATA_X1    0x25

volatile bool running = true;

void signalHandler(int signum) {
    running = false;
}

bool spiInit() {
    if (wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) < 0) {
        std::cerr << "SPI setup failed!" << std::endl;
        return false;
    }
    return true;
}

uint8_t spiReadRegister(uint8_t reg) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg | 0x80), 0x00 };
    uint8_t rx[2] = {0};
    wiringPiSPIDataRW(SPI_CHANNEL, tx, 2);
    return tx[1];
}

void spiWriteRegister(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7F), value };
    wiringPiSPIDataRW(SPI_CHANNEL, tx, 2);
}

// 读取连续寄存器数据
bool spiReadBytes(uint8_t reg, uint8_t* buffer, int length) {
    if (length > 255) return false; // 限长
    uint8_t tx[2];
    memset(tx, 0, sizeof(tx));
    tx[0] = reg | 0x80;
    int result = wiringPiSPIDataRW(SPI_CHANNEL, tx, length + 1);
    memcpy(buffer, &tx[1], length);
    return true;
}

void selectBank(uint8_t bank) {
    spiWriteRegister(REG_BANK_SEL, bank);
}

void initIMU() {
    // 设备复位
    selectBank(0);
    spiWriteRegister(DEVICE_CONFIG, 0x01);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    selectBank(0);
    // 关闭待机，低噪模式启用陀螺仪和加速度计，温度传感器启用
    spiWriteRegister(PWR_MGMT0, 0x0F);

    // 设陀螺仪和加速度计 ODR均为1kHz，且量程为陀螺仪±2000dps，加速度计±8g
    spiWriteRegister(GYRO_CONFIG0, (0 << 5) | 0x06); // Gyro ODR=1kHz, FS=±2000dps
    spiWriteRegister(ACCEL_CONFIG0, (1 << 5) | 0x06); // Accel ODR=1kHz, FS=±8g

    // // 配置陀螺仪抗混叠滤波器 （AAF）带宽为997Hz，关闭Notch滤波器
    // selectBank(1);
    // uint8_t val = spiReadRegister(0x0B);
    // val = (val | 0x01) & ~0x02; // Bit0置1禁用Notch, Bit1清0启用AAF
    // spiWriteRegister(0x0B, val);
    // spiWriteRegister(0x0C, 21 & 0x3F); // GYRO_AAF_DELT=21 (对应997Hz)
    // uint16_t aaf_deltsqr = 440;
    // spiWriteRegister(0x0D, aaf_deltsqr & 0xFF);
    // uint8_t tmp = spiReadRegister(0x0E);
    // tmp = (tmp & 0x0F) | (6 << 4); // GYRO_AAF_BITSHIFT=6
    // tmp = (tmp & 0xF0) | ((aaf_deltsqr >> 8) & 0x0F);
    // spiWriteRegister(0x0E, tmp);

    // // 加速度计AAF配置同陀螺仪，997Hz，启用AAF
    // selectBank(2);
    // uint8_t val_acc = spiReadRegister(0x04);
    // val_acc = (val_acc & 0x81) | ((21 <<1) & 0x7E); // ACCEL_AAF_DELT=21 设置，启用AAF(bit0=0)
    // spiWriteRegister(0x04, val_acc);
    // uint16_t acc_aaf_deltsqr = 440;
    // spiWriteRegister(0x05, acc_aaf_deltsqr & 0xFF);
    // tmp = spiReadRegister(0x06);
    // tmp = (tmp & 0x0F) | (6 << 4);
    // tmp = (tmp & 0xF0) | ((acc_aaf_deltsqr >> 8) & 0x0F);
    // spiWriteRegister(0x06, tmp);

    // 回到Bank0
    selectBank(0);
    spiWriteRegister(FIFO_CONFIG, 0x00); // 关闭FIFO，避免数据混乱

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

int16_t combineBytes(uint8_t highByte, uint8_t lowByte) {
    return (int16_t)(((int16_t)highByte << 8) | (int16_t)lowByte);
}

int main() {
    signal(SIGINT, signalHandler);

    if (wiringPiSetupGpio() < 0) {
        std::cerr << "Setup wiringPi GPIO failed!" << std::endl;
        return -1;
    }
    if (!spiInit()) return -1;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    initIMU();

    // 在初始化后添加WHO_AM_I检查
    uint8_t whoami = spiReadRegister(0x75);
    std::cerr << "WHO_AM_I: " << std::hex << (int)whoami << std::endl;

    std::ofstream fout("imu_data.txt");
    if (!fout.is_open()) {
        std::cerr << "Failed to open imu_data.txt for writing." << std::endl;
        return -1;
    }
    fout << "Timestamp_us\tGyroX\tGyroY\tGyroZ\tAccelX\tAccelY\tAccelZ\tTemperature_C\n";

    uint8_t buf[READ_LEN];
    auto nextTime = std::chrono::steady_clock::now();
    auto startTime = nextTime;
    std::cout << startTime.time_since_epoch().count() << std::endl;
    while (running) {
        uint8_t intStatus = spiReadRegister(INT_STATUS);
        if (intStatus & (1<<3)) {
            nextTime += std::chrono::microseconds(800);
            uint64_t ts = std::chrono::duration_cast<std::chrono::microseconds>(
                          std::chrono::steady_clock::now() - startTime)
                          .count();
            // if (!spiReadBytes(REG_TEMP_DATA1, buf, READ_LEN)) {
            //     std::cerr << "SPI read failed" << std::endl;
            //     continue;
            // }
            // int16_t temp_raw = combineBytes(buf[0], buf[1]);
            // int16_t ax = combineBytes(buf[2], buf[3]);
            // int16_t ay = combineBytes(buf[4], buf[5]);
            // int16_t az = combineBytes(buf[6], buf[7]);
            // int16_t gx = combineBytes(buf[8], buf[9]);
            // int16_t gy = combineBytes(buf[10], buf[11]);
            // int16_t gz = combineBytes(buf[12], buf[13]);

            uint8_t tempBuf[2];
            uint8_t accelBuf[6];
            uint8_t gyroBuf[6];
    
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

            static uint64_t dbg_cnt = 0;
            if (dbg_cnt++ % 100 == 0) {
                float gyroScale = 2000.0f / 32768.0f; // ±2000dps 对应的缩放因子
                float accelScale = 8.0f / 32768.0f; // ±8g 对应的缩放因子
                std::cout << "Timestamp: " << ts << ", "
                          << "Gyro: (" << gx * gyroScale << ", " << gy * gyroScale << ", " << gz * gyroScale << "), "
                          << "Accel: (" << ax * accelScale << ", " << ay * accelScale << ", " << az * accelScale << "), "
                          << "Temp: " << temp_raw / 132.48f + 25.f << ", "
                          << std::endl;
            }
    
            // 温度转换，数据手册建议公式：Temp(C) = (temp_raw / 132.48) + 25
            // float temperature = temp_raw / 132.48f + 25.f;
            // float gyroX = gx / 131.0f;
            // float accX = ax / 16384.0f;
            fout << ts << "\t" << gx << "\t" << gy << "\t" << gz << "\t"
                 << ax << "\t" << ay << "\t" << az << "\t" << temp_raw << "\n";
            
            std::this_thread::sleep_until(nextTime);
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(3));
            continue;
        }
    }
    fout.flush();
    fout.close();
    return 0;
}