// gcc -o imu_read_register imu_read_register.cpp -lgpiod

#include <gpiod.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define SPI_DEVICE "/dev/spidev0.0"     // ls /dev/spidev*
#define REGISTER_ADDRESS 0x75           // IMU 的 WHO_AM_I 寄存器地址

int main() {
    int spi_fd;
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 100000;
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];
    struct spi_ioc_transfer spi_transfer = {0};

    // 打开 SPI 设备
    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) {
        perror("Failed to open SPI device");
        return -1;
    }

    // 设置 SPI 模式和速度
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    // 准备发送和接收的数据
    tx_buf[0] = REGISTER_ADDRESS | 0x80; // 设置读位
    tx_buf[1] = 0x00; // 通常发送 0x00，保持时钟运行

    spi_transfer.tx_buf = (unsigned long)tx_buf;
    spi_transfer.rx_buf = (unsigned long)rx_buf;
    spi_transfer.len = 2;
    spi_transfer.speed_hz = speed;
    spi_transfer.bits_per_word = bits;

    // 执行 SPI 数据传输
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer) < 0) {
        perror("Failed to communicate with SPI device");
        close(spi_fd);
        return -1;
    }

    // 打印读取的寄存器值
    printf("Register 0x%02X value: 0x%02X\n", REGISTER_ADDRESS, rx_buf[1]);

    // 关闭 SPI 设备
    close(spi_fd);
    return 0;
}