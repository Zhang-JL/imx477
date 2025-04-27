#include <gpiod.h>
#include <iostream>
#include <csignal>
#include <unistd.h>

volatile bool running = true;
int counter = 0;

void signalHandler(int signum) {
    running = false;
}

int main() {
    const char *chipname = "gpiochip0"; // Raspberry Pi 通常是 gpiochip0
    const unsigned int line_num = 18;   // BCM GPIO18

    // 处理 Ctrl+C 信号
    std::signal(SIGINT, signalHandler);

    // 打开 GPIO 芯片
    gpiod_chip *chip = gpiod_chip_open_by_name(chipname);
    if (!chip) {
        std::cerr << "无法打开 GPIO 芯片" << std::endl;
        return 1;
    }

    // 获取指定 GPIO 行
    gpiod_line *line = gpiod_chip_get_line(chip, line_num);
    if (!line) {
        std::cerr << "无法获取 GPIO" << std::endl;
        gpiod_chip_close(chip);
        return 1;
    }

    // 请求输入 + 监听下降沿事件
    if (gpiod_line_request_falling_edge_events(line, "gpio_interrupt") < 0) {
        std::cerr << "无法请求中断事件" << std::endl;
        gpiod_chip_close(chip);
        return 1;
    }

    std::cout << "开始监听 GPIO18 下降沿中断，按 Ctrl+C 退出..." << std::endl;

    while (running) {
        struct gpiod_line_event event;
        int ret = gpiod_line_event_wait(line, nullptr); // 阻塞等待事件

        if (ret < 0) {
            std::cerr << "等待事件失败" << std::endl;
            break;
        } else if (ret == 0) {
            continue; // 超时（但我们没设置超时，所以基本不会发生）
        }

        if (gpiod_line_event_read(line, &event) == 0) {
            counter++;
            std::cout << "中断 #" << counter << " - 时间戳: " << event.ts.tv_sec << "s" << std::endl;
        }
    }

    std::cout << "程序退出，共触发中断次数: " << counter << std::endl;

    gpiod_line_release(line);
    gpiod_chip_close(chip);
    return 0;
}
