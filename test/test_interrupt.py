import RPi.GPIO as GPIO
import time

# 设置 GPIO 模式为 BCM
GPIO.setmode(GPIO.BCM)

# 定义 GPIO 引脚
GPIO_PIN = 18

# 设置 GPIO18 为输入模式，并启用上拉电阻
GPIO.setup(GPIO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# 初始化中断计数器
interrupt_count = 0

# 中断回调函数
def gpio_callback(channel):
    global interrupt_count
    interrupt_count += 1
    print(f"中断触发！GPIO{channel} 状态：{GPIO.input(channel)}，总中断次数：{interrupt_count}")

# 添加中断检测，检测下降沿（falling edge）
GPIO.add_event_detect(GPIO_PIN, GPIO.FALLING, callback=gpio_callback, bouncetime=20)

print("等待 GPIO18 中断信号，按 Ctrl+C 退出")

try:
    # 主循环
    while True:
        time.sleep(1)  # 保持脚本运行
except KeyboardInterrupt:
    print(f"退出程序，总共接收到的中断次数：{interrupt_count}")
finally:
    # 清理 GPIO 设置
    GPIO.cleanup()