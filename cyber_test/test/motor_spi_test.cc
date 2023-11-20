#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <csignal>
#include <fstream>
#include <iostream>

#include "include/timer.h"
#include "spi_can/motor_spi.h"

#define CURR_CLOSE_TEST  // 电流环'闭环'测试
// #define CURR_OPEN_TEST  // 电流环'开环'测试
// #define POSTEST        // 位置环'开环'测试
// #define USENONBLOCK    // 开启后使用非阻塞式通讯，关闭后使用阻塞式通讯
// #define PRINTTIMEOUT   // 开启后打印多少个Timeout接收到信号
// #define SPLITSENDRECV  // 将接收与发送线程分离开
#define PRINT_CAN_SEND

namespace {
const float kFrequency = 500;  // 控制频率(Hz)，数值允许0.5~500
const int kMotorCount = 1;     // 电机总数
const float kKp = 20.0;        // 伺服跟踪Kp
const float kKd = 0.01;        // 伺服跟踪Kd
const bool kSaveData = true;   // 是否保存测试数据

const int kWaveForm = 1;              // 0使用方波信号，1使用正弦波信号
const double kSquareCycle = 1;        // 方波周期（s）
const double kSquareAmplitude = 1.0;  // 方波幅值（A）
const float kSinAmplitude = 2.0;      // sin幅值（A）
const float kSinCycle = 0.5;          // sin周期（s）
const float kSinOmega = 2.f * M_PI / kSinCycle;
const int kMotorPrint = 0;

volatile std::sig_atomic_t stop_signal;
}  // namespace

MotorSpi m_c;

int main() {
  std::signal(SIGINT, [](int ctrl_c) { stop_signal = ctrl_c; });
  std::ofstream log;
  log.open("../data/data.txt");

  float set_pos[12] = {0};
  float set_vel[12] = {0};
  float set_cur[12] = {0};

  int iteration = 0;
  double time_now = 0.0;
  bool enable_send = false;
  unsigned int sleep_count = 1820;
  int send_iterval = 500 / kFrequency;

  m_c.init_bus();
  for (int i = 0; i < kMotorCount; ++i) {
    m_c.set_en(i, true);
    std::cout << "id: " << i << " enabled" << std::endl;
  }
  std::cout << "CAN communication is setup successfully." << std::endl
            << "WARNING: Make sure the motor is enabled." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

#ifdef SPLITSENDRECV
  m_c.init_recv_thread();
#endif  // SPLITSENDRECV

  Timer timer;
  timer.Start();

#ifndef POSTEST
  float fai_bias[12] = {0};
  for (int n = 0; n < 10; ++n) {
    for (int i = 0; i < kMotorCount; ++i) {
      m_c.set_value(i, 0, 0.f);
    }
    usleep(5000);
  }
  for (int i = 0; i < kMotorCount; ++i) {
    time_now = timer.GetSeconds();
    m_c.motors[i].position_zero = m_c.motors[i].position_real;
    fai_bias[i] = -kSinOmega * time_now;
    // std::cout << "id: " << i << " fai_bias[i]: " << fai_bias[i] << std::endl;
  }
#endif  // POSTEST

  while (true) {
    time_now = timer.GetSeconds();
    enable_send = iteration % send_iterval == 0;

    if (kWaveForm == 0) {
      int mod = time_now / kSquareCycle;
      if (mod % 2 == 0) {
        for (int i = 0; i < kMotorCount; ++i) {
          set_cur[i] = -kSquareAmplitude;
        }
      } else {
        for (int i = 0; i < kMotorCount; ++i) {
          set_cur[i] = kSquareAmplitude;
        }
      }
      // printf("Now %6.4f, SetCurr %2.6f\n", time_now, set_cur[0]);
    } else if (kWaveForm == 1) {
      for (int i = 0; i < kMotorCount; ++i) {
        set_pos[i] = kSinAmplitude * std::sin(kSinOmega * time_now + fai_bias[i]);
        set_vel[i] = kSinAmplitude * kSinOmega * std::cos(kSinOmega * time_now + fai_bias[i]);
        set_cur[i] = kSinAmplitude * std::sin(kSinOmega * time_now + fai_bias[i]);
      }
      printf("Now %6.4f, SetCurr %2.6f\n", time_now, set_cur[0]);
    } else {
      printf("[Error] Undefined wave form!\n");
      return -1;
    }

#ifdef POSTEST
    if (enable_send) {
      for (int i = 0; i < kMotorCount; ++i) {
        m_c.set_value(i, 2, set_pos[i]);
      }
      printf("Now %6.4f, SetPos %2.6f rad, ReadPos %2.6f rad, ReadVel %2.6f, ReadCurr %2.6f\n", time_now,
             set_pos[kMotorPrint], m_c.motors[kMotorPrint].position_real, m_c.motors[kMotorPrint].velocity_real,
             m_c.motors[kMotorPrint].current_real);
    }
#endif  // POSTEST

#ifdef CURR_OPEN_TEST
    if (enable_send) {
      for (int i = 0; i < kMotorCount; ++i) {
        m_c.set_value(i, 0, set_cur[i]);
      }
      printf("Now %6.4f, SetCurr %2.6f A, ReadCurr %2.6f A, ReadPos %2.6f, ReadVel %2.6f\n", time_now,
             set_pos[kMotorPrint], m_c.motors[kMotorPrint].current_real, m_c.motors[kMotorPrint].position_real,
             m_c.motors[kMotorPrint].velocity_real);
    }
#endif  // CURR_OPEN_TEST

#ifdef CURR_CLOSE_TEST
    if (enable_send) {
      for (int i = 0; i < kMotorCount; ++i) {
        set_cur[i] =
            kKp * (set_pos[i] - m_c.motors[i].position_real) + kKd * (set_vel[i] - 0.5 * m_c.motors[i].velocity_real);
        m_c.set_value(i, 0, set_cur[i]);
      }
      printf("Now %6.4f, SetPos %2.6f, ReadPos %2.6f, SetVel %2.6f, ReadVel %2.6f, SetCurr %2.6f, ReadCurr %2.6f\n",
             time_now, set_pos[kMotorPrint], m_c.motors[kMotorPrint].position_real, set_vel[kMotorPrint],
             m_c.motors[kMotorPrint].velocity_real, set_cur[kMotorPrint], m_c.motors[kMotorPrint].current_real);
    }
#endif  // CURR_CLOSE_TEST

    if (enable_send && kSaveData) {
      log << time_now << " ";
      for (int i = 0; i < kMotorCount; ++i) {
        log << set_pos[i] << " " << m_c.motors[i].position_real << " " << set_vel[i] << " "
            << m_c.motors[i].velocity_real << " " << set_cur[i] << " " << m_c.motors[i].current_real << " ";
      }
      log << "\n";
    }

    if (stop_signal) {
      log.close();
      std::cout << std::endl;
      for (int i = 0; i < kMotorCount; ++i) {
        m_c.set_en(i, false);
        usleep(10);
        std::cout << "id: " << i << " disabled" << std::endl;
      }
      m_c.de_init_bus();
      return 0;
    }

    iteration++;
    usleep(sleep_count);
  }
  return 0;
}