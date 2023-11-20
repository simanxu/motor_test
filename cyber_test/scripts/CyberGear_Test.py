import time
from DrEmpower_CyberGear import *

start_time = time.time()  # 记录程序开始时间
TEST_TIME = 0.5

def test_time():
    # 获取当前时间戳，单位为秒
    current_time = time.time()
    # 打印当前时间
    print("当前时间（秒）%.4f：" % (current_time - start_time))


def test():
    dt = 0.002  # 定义时间间隔为2毫秒
    t = 0.0
    motor_enable(1)
    while t < TEST_TIME:
        # test_time()
        set_speed(1, 0.5, 27)
        t = t + dt
        time.sleep(dt)


if __name__ == "__main__":
    test()
