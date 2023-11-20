import time
import threading
import inspect
import math

from DrEmpower_CyberGear import *


TEST_TIME = 2.0
DT = 0.002
ID = 1
# mode = 0: 运控模式; mode = 1: 位置模式; mode = 2: 速度模式; mode = 3: 电流模式
MODE = 1

file_path = os.path.join(os.path.dirname(os.getcwd()), "motor_test/cyber_test/data/data.txt")
print(file_path)
# 打开文件并写入数据
file = open(file_path, "w")
zcmd = 0.0


def init(id = -1, mode = -1):
    motor_enable(id)
    clear_error(id)
    set_zero_position(id)
    set_mode(id, mode)


def finalize(id = -1):
    motor_estop(id)
    file.close()


def position_control(t, amplitude = 2.0, period = 1.0):
    """
       @brief 电机位置控制测试
       通过设置正弦曲线的幅值(amplitude)和周期(period)来控制电机的位置指令
    """
    # print(f"{inspect.currentframe().f_code.co_name} is called {t:.4f}")
    omega = 2 * math.pi / period
    pos = amplitude * math.sin(omega * t)
    vel = amplitude * omega * math.cos(omega * t)
    set_angle(ID, pos, vel)
    pos_vel = get_state(ID)
    vol_cur = get_volcur(ID)
    line = "{:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}\n".format(pos, pos_vel[0], vel, pos_vel[1], zcmd, vol_cur[1])
    file.write(line)

def velocity_control(t, amplitude = 2.0, period = 1.0):
    """
       @brief 电机速度控制测试
       通过设置正弦曲线的幅值(amplitude)和周期(period)来控制电机的速度指令
    """
    # print(f"{inspect.currentframe().f_code.co_name} is called {t:.4f}")
    omega = 2 * math.pi / period
    pos = amplitude * math.sin(omega * t)
    vel = amplitude * omega * math.cos(omega * t)
    set_speed(ID, vel)
    pos_vel = get_state(ID)
    vol_cur = get_volcur(ID)
    line = "{:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}".format(pos, pos_vel[0], vel, pos_vel[1], zcmd, vol_cur[1])
    file.write(line)


def current_control(t, amplitude = 2.0, period = 1.0):
    """
       @brief 电机电流控制测试
       通过设置正弦曲线的幅值(amplitude)和周期(period)来控制电机的电流指令
    """
    # print(f"{inspect.currentframe().f_code.co_name} is called {t:.4f}")
    omega = 2 * math.pi / period
    cur = amplitude * math.sin(omega * t)
    set_torque(ID, cur)
    pos_vel = get_state(ID)
    vol_cur = get_volcur(ID)
    line = "{:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}".format(zcmd, pos_vel[0], zcmd, pos_vel[1], zcmd, vol_cur[1])
    file.write(line)


def motion_control(t, amplitude = 2.0, period = 1.0, kp = 10, kd = 0.1):
    """
       @brief 电机阻抗控制测试
       通过设置正弦曲线的幅值(amplitude)和周期(period)来控制电机的位置、速度，生成电流指令
    """
    # print(f"{inspect.currentframe().f_code.co_name} is called {t:.4f}")
    omega = 2 * math.pi / period
    pos = amplitude * math.sin(omega * t)
    vel = amplitude * omega * math.cos(omega * t)
    pos_vel = get_state(ID)
    vol_cur = get_volcur(ID)
    tff = kp * (pos - pos_vel[0]) + kd * (vel - pos_vel[1])
    impedance_control(ID, pos, vel, tff=tff, kp=kp, kd=kd)
    line = "{:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}".format(pos, pos_vel[0], vel, pos_vel[1], tff, vol_cur[1])
    file.write(line)


def callback(t):
    print(f"{inspect.currentframe().f_code.co_name} is called {t:.4f} and {time.time():.4f}")
    if MODE == 0:
        motion_control(t)
    elif MODE == 1:
        position_control(t)
    elif MODE == 2:
        velocity_control(t)
    elif MODE == 3:
        current_control(t)
    else:
        print("Error: Invalid motor mode.")


def timer(dt = 0.002):
    """
        @brief 线程控制函数
        通过设置线程的周期(dt)来控制回调函数的召回
    """
    start_time = time.time()
    elapsed_time = 0
    while elapsed_time <= TEST_TIME:
        callback(elapsed_time)
        elapsed_time = time.time() - start_time
        remaining_time = dt - elapsed_time % dt
        time.sleep(remaining_time)


if __name__ == "__main__":
    # init(ID, MODE)
    timer(DT)
    # finalize()

