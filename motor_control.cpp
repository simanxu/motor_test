/**
 * @author [xujiafeng]
 * @email [2235797879@qq.com]
 * @create date 2023-06-15 14:31:12
 * @modify date 2023-06-15 14:31:12
 * @desc [motor control test porgram]
 */
#include "motor_control.h"

#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <algorithm>
#include <csignal>
#include <fstream>
#include <iostream>

#define CURR_CLOSE_TEST  // 电流环'闭环'测试
// #define CURR_OPEN_TEST  // 电流环'开环'测试
// #define POSTEST        // 位置环'开环'测试
// #define USENONBLOCK  // 开启后使用非阻塞式通讯，关闭后使用阻塞式通讯
// #define PRINTTIMEOUT   // 开启后打印多少个Timeout接收到信号
// #define SPLITSENDRECV  // 将接收与发送线程分离开
// #define PRINT_CAN_SEND

namespace {
const float kFrequency = 500;      // 控制频率(Hz)，数值允许0.1~500
const int kMotorStart = 0;         // 电机开始的编号
const int kMotorFinal = 12;        // 电机结束的编号（必须大于开始编号）
const float kKp = 50;              // 伺服跟踪Kp
const float kKd = 0.125;           // 伺服跟踪Kd
const int kCanRecvTimeout = 5000;  // 非阻塞式通讯模式下，接收超时的次数（丢包严重则改大此数）
const bool kSaveData = true;       // 是否保存测试数据

const int kWaveForm = 1;              // 0使用方波信号，1使用正弦波信号，2使用单方向电流信号
const double kSquareCycle = 1;        // 方波周期（s）
const double kSquareAmplitude = 2.0;  // 方波幅值（A）
const float kSinAmplitude = 2;        // sin幅值（A）
const float kSinCycle = 1;            // sin周期（s）
const float kSinOmega = 2.f * M_PI / kSinCycle;
const int kMotorPrint = 1;

const double kKCurrent = 1e-3;  // 斜坡电流的斜率
const double kMaxCurrent = 2;   // 斜坡电流的最大值

const float kGearRatio = 8.f;
const float kTorqueCoefficient = 0.21f;                     // Nm/A
const float kPosDri2Jnt = 1.f / kGearRatio;                 // 1 rad -> 1/8 rad
const float kVelDri2Jnt = 1.f / kGearRatio / (2.f * M_PI);  // 1 rps -> 1/(2pi) rad/s -> 1/(2pi)/8 rad/s
const float kTauDri2Jnt = kGearRatio * kTorqueCoefficient;  // 1 A -> 8 A -> 8*ke Nm
const float kKneeRatio = 1.f / 0.72266f;
const float kCurrentLimit = 10.f;

volatile std::sig_atomic_t stop_signal;
}  // namespace

MotorControl m_c;

int main() {
  std::signal(SIGINT, [](int ctrl_c) { stop_signal = ctrl_c; });
  std::ofstream log;
  log.open("../plot/data.txt");

  float set_pos[12] = {0};
  float set_vel[12] = {0};
  float set_tau[12] = {0};

  int iteration = 0;
  double time_now = 0.0;
  bool enable_send = false;
  unsigned int sleep_count = 1820;
  int send_iterval = 500 / kFrequency;
  std::cout << "[CYCLE] Count of send_iterval is: " << send_iterval << std::endl;

  m_c.init_bus();
  for (int i = kMotorStart; i < kMotorFinal; ++i) {
    m_c.set_en(i, true);
    std::cout << "motor id: " << i << " has send enabled signal." << std::endl;
  }
  // std::cout << "CAN communication is setup successfully." << std::endl
  //           << "WARNING: Make sure the motor is enabled." << std::endl
  //           << "Press Enter to continue..." << std::endl;
  // std::cin.ignore();

#ifdef SPLITSENDRECV
  m_c.init_recv_thread();
#endif  // SPLITSENDRECV

#ifndef POSTEST
  float fai_bias[12] = {0};
  for (int i = kMotorStart; i < kMotorFinal; ++i) {
    m_c.set_value(i, 0, 0.f);
    usleep(500);
  }
  for (int i = kMotorStart; i < kMotorFinal; ++i) {
    m_c.motors[i].position_zero = -m_c.motors[i].position_real;
    // float sin_fai = m_c.motors[i].position_real / kSinAmplitude;
    // fai_bias[i] = std::asin(sin_fai);
    // std::cout << "id: " << i << " fai_bias: " << fai_bias[i] << std::endl;
  }
#endif  // POSTEST

  Timer timer;
  timer.start();

  while (true) {
    time_now = timer.getSeconds();
    enable_send = iteration % send_iterval == 0;

    if (kWaveForm == 0) {
      int mod = time_now / kSquareCycle;
      if (mod % 2 == 0) {
        for (int i = kMotorStart; i < kMotorFinal; ++i) {
          set_tau[i] = -kSquareAmplitude;
        }
      } else {
        for (int i = kMotorStart; i < kMotorFinal; ++i) {
          set_tau[i] = kSquareAmplitude;
        }
      }
      // printf("Now %6.4f, SetTau %2.6f\n", time_now, set_tau[0]);
    } else if (kWaveForm == 1) {
      for (int i = kMotorStart; i < kMotorFinal; ++i) {
        set_pos[i] = kSinAmplitude * std::sin(kSinOmega * time_now + fai_bias[i]);
        set_vel[i] = kSinAmplitude * kSinOmega * std::cos(kSinOmega * time_now + fai_bias[i]);
        set_tau[i] = kSinAmplitude * std::sin(kSinOmega * time_now + fai_bias[i]);
      }
      // printf("Now %6.4f, %2.5f, SetTau %2.6f\n", time_now, kSinAmplitude, set_tau[0]);
    } else if (kWaveForm == 2) {
      for (int i = kMotorStart; i < kMotorFinal; ++i) {
        set_tau[i] = kKCurrent * iteration;
        if (set_tau[i] > kMaxCurrent) {
          set_tau[i] = kMaxCurrent;
        }
      }
    } else {
      printf("[Error] Undefined wave form!\n");
      return -1;
    }

#ifdef POSTEST
    if (enable_send) {
      for (int i = kMotorStart; i < kMotorFinal; ++i) {
        m_c.set_value(i, 2, set_pos[i]);
      }
      printf(
          "Now %6.4f, SetTime %2.6f, GetTime %2.6f, SetPos %2.6f rad, ReadPos %2.6f rad, ReadVel %2.6f, ReadCurr "
          "%2.6f\n",
          time_now, m_c.time_set_, m_c.time_get_, set_pos[kMotorPrint], m_c.motors[kMotorPrint].position_real,
          m_c.motors[kMotorPrint].velocity_real, m_c.motors[kMotorPrint].torque_real);
    }
#endif  // POSTEST

#ifdef CURR_OPEN_TEST
    if (enable_send) {
      for (int i = kMotorStart; i < kMotorFinal; ++i) {
        m_c.set_value(i, 0, set_tau[i]);
      }
      printf(
          "Now %6.4f, SetTime %2.6f, GetTime %2.6f, SetTau %2.6f A, ReadCurr %2.6f A, ReadPos %2.6f, ReadVel %2.6f\n",
          time_now, m_c.time_set_, m_c.time_get_, set_tau[kMotorPrint], m_c.motors[kMotorPrint].torque_real,
          m_c.motors[kMotorPrint].position_real, m_c.motors[kMotorPrint].velocity_real);
    }
#endif  // CURR_OPEN_TEST

#ifdef CURR_CLOSE_TEST
    if (enable_send) {
      for (int i = kMotorStart; i < kMotorFinal; ++i) {
        float cur = kKp * (set_pos[i] - m_c.motors[i].position_real) + kKd * (set_vel[i] - m_c.motors[i].velocity_real);
        set_tau[i] = std::clamp(cur, -kCurrentLimit, kCurrentLimit);
        m_c.set_value(i, 0, set_tau[i]);
      }
      printf(
          "Now %6.4f, SetTime %2.6f, GetTime %2.6f, SetPos %2.6f, ReadPos %2.6f, SetVel %2.6f, ReadVel %2.6f, SetTau "
          "%2.6f, ReadTau %2.6f\n",
          time_now, m_c.time_set_, m_c.time_get_, set_pos[kMotorPrint], m_c.motors[kMotorPrint].position_real,
          set_vel[kMotorPrint], m_c.motors[kMotorPrint].velocity_real, set_tau[kMotorPrint],
          m_c.motors[kMotorPrint].torque_real);
    }
#endif  // CURR_CLOSE_TEST

    if (enable_send && kSaveData) {
      log << time_now << " ";
      for (int i = kMotorStart; i < kMotorFinal; ++i) {
        log << set_pos[i] << " " << m_c.motors[i].position_real << " " << set_vel[i] << " "
            << m_c.motors[i].velocity_real << " " << set_tau[i] << " " << m_c.motors[i].torque_real << " ";
      }
      log << m_c.time_set_ << " " << m_c.time_get_ << "\n";
    }

    if (stop_signal) {
      log.close();
      std::cout << std::endl;
      for (int i = kMotorStart; i < kMotorFinal; ++i) {
        m_c.set_en(i, false);
        usleep(10);
        std::cout << "motor id: " << i << " has send disabled signal." << std::endl;
      }
      m_c.de_init_bus();
      return 0;
    }

    iteration++;
    usleep(sleep_count);
  }
  return 0;
}

MotorControl::MotorControl() {
  motors[0].id = 0x01;
  motors[0].ch = 0;
  motors[1].id = 0x02;
  motors[1].ch = 0;
  motors[2].id = 0x03;
  motors[2].ch = 0;
  motors[3].id = 0x04;
  motors[3].ch = 0;
  motors[4].id = 0x05;
  motors[4].ch = 0;
  motors[5].id = 0x06;
  motors[5].ch = 0;

  motors[6].id = 0x07;
  motors[6].ch = 1;
  motors[7].id = 0x08;
  motors[7].ch = 1;
  motors[8].id = 0x09;
  motors[8].ch = 1;
  motors[9].id = 0x0A;
  motors[9].ch = 1;
  motors[10].id = 0x0B;
  motors[10].ch = 1;
  motors[11].id = 0x0C;
  motors[11].ch = 1;

  is_inited = false;

  for (int i = 0; i < 12; ++i) {
    motors[i].direction = 1;
    motors[i].pos_ratio = kPosDri2Jnt;
    motors[i].vel_ratio = kVelDri2Jnt;
    motors[i].tau_ratio = kTauDri2Jnt;
    motors[i].position_zero = 0.f;
    motors[i].position_set = 0.f;
    motors[i].velocity_set = 0.f;
    motors[i].current_set = 0.f;
    motors[i].ke = kTorqueCoefficient;
  }

  // define motor position zeros
  // motors[0].direction = -1;
  // motors[0].position_zero = 0.090175;

  // motors[1].direction = -1;
  // motors[1].position_zero = -2.474798;

  // motors[2].pos_ratio = kPosDri2Jnt / kKneeRatio;
  // motors[2].vel_ratio = kVelDri2Jnt / kKneeRatio;
  // motors[2].tau_ratio = kTauDri2Jnt * kKneeRatio;
  // motors[2].direction = -1;
  // motors[2].position_zero = 2.398147;

  // motors[3].direction = -1;
  // motors[3].position_zero = 0.189506;

  // motors[4].direction = 1;
  // motors[4].position_zero = -2.392916;

  // motors[5].pos_ratio = kPosDri2Jnt / kKneeRatio;
  // motors[5].vel_ratio = kVelDri2Jnt / kKneeRatio;
  // motors[5].tau_ratio = kTauDri2Jnt * kKneeRatio;
  // motors[5].direction = 1;
  // motors[5].position_zero = 2.785435;

  // motors[6].direction = 1;
  // motors[6].position_zero = 0.4896335;

  // motors[7].direction = -1;
  // motors[7].position_zero = -2.454663;

  // motors[8].pos_ratio = kPosDri2Jnt / kKneeRatio;
  // motors[8].vel_ratio = kVelDri2Jnt / kKneeRatio;
  // motors[8].tau_ratio = kTauDri2Jnt * kKneeRatio;
  // motors[8].direction = -1;
  // motors[8].position_zero = 2.882577;

  // motors[9].direction = 1;
  // motors[9].position_zero = -0.261368;

  // motors[10].direction = 1;
  // motors[10].position_zero = -2.876054;

  // motors[11].pos_ratio = kPosDri2Jnt / kKneeRatio;
  // motors[11].vel_ratio = kVelDri2Jnt / kKneeRatio;
  // motors[11].tau_ratio = kTauDri2Jnt * kKneeRatio;
  // motors[11].direction = 1;
  // motors[11].position_zero = 2.752557;

  memset(&rsv_frame0, 0, sizeof(struct can_frame));
  memset(&rsv_frame1, 0, sizeof(struct can_frame));
  init_bus();
  timer_.start();
  time_get_ = 0.0;
  time_set_ = 0.0;
}

MotorControl::~MotorControl() {
  de_init_bus();
  if (can_recv_thread_.joinable()) {
    can_recv_thread_.join();
  }
}

int MotorControl::init_bus() {
  system("sudo ip link set can0 up type can bitrate 1000000");
  system("sudo ip link set can1 up type can bitrate 1000000");

  system("sudo ifconfig can0 txqueuelen 65536");
  system("sudo ifconfig can1 txqueuelen 65536");

// 1.Create socket
#ifdef USENONBLOCK
  can_s0 = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
#else
  can_s0 = socket(PF_CAN, SOCK_RAW, CAN_RAW);  // |SOCK_NONBLOCK
#endif
  if (can_s0 < 0) {
    return 1;
  }

#ifdef USENONBLOCK
  can_s1 = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
#else
  can_s1 = socket(PF_CAN, SOCK_RAW, CAN_RAW);  // |SOCK_NONBLOCK
#endif
  if (can_s1 < 0) {
    return 1;
  }

  // 2.Specify can0 device
  strcpy(ifr0.ifr_name, "can0");
  ret = ioctl(can_s0, SIOCGIFINDEX, &ifr0);
  if (ret < 0) {
    return 1;
  }
  strcpy(ifr1.ifr_name, "can1");
  ret = ioctl(can_s1, SIOCGIFINDEX, &ifr1);
  if (ret < 0) {
    return 1;
  }

  // 3.Bind the socket to can
  addr0.can_family = PF_CAN;
  addr0.can_ifindex = ifr0.ifr_ifindex;
  ret = bind(can_s0, (struct sockaddr*)&addr0, sizeof(addr0));
  if (ret < 0) {
    return 1;
  }

  addr1.can_family = PF_CAN;
  addr1.can_ifindex = ifr1.ifr_ifindex;
  ret = bind(can_s1, (struct sockaddr*)&addr1, sizeof(addr1));
  if (ret < 0) {
    return 1;
  }

  // fcntl(can_s0, F_SETFL, FNDELAY);  // 用于设置read函数的非阻塞
  // fcntl(can_s1, F_SETFL, FNDELAY);  // 用于设置read函数的非阻塞

  // 4.Define receive rules
  // struct can_filter rfilter[1];
  // rfilter[0].can_id = 0x123;
  // rfilter[0].can_mask = CAN_SFF_MASK;
  // setsockopt(can_s0, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
  // setsockopt(can_s1, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  is_inited = true;
  return 0;
}

void MotorControl::init_recv_thread() {
  can_recv_thread_ = std::thread(&MotorControl::RecvMotorStatus, this);
  can_recv_thread_.detach();
}

void MotorControl::send_loop() {}

int MotorControl::send_by_bus(unsigned char ch, unsigned short id, unsigned char* data, unsigned char len) {
  int ii;
  struct can_frame frame;
  memset(&frame, 0, sizeof(struct can_frame));
  frame.can_id = id;
  frame.can_dlc = len;
  for (ii = 0; ii < len; ++ii) frame.data[ii] = data[ii];
  switch (ch) {
    case 1:
      nbytes = write(can_s1, &frame, sizeof(frame));
      break;
    default:
      nbytes = write(can_s0, &frame, sizeof(frame));
      break;
  }

  if (nbytes != sizeof(frame)) return 1;

  return 0;
}

int MotorControl::send_by_bus(unsigned char ch, unsigned short id) {
  struct can_frame frame;
  memset(&frame, 0, sizeof(struct can_frame));
  frame.can_id = id;
  frame.can_id += CAN_RTR_FLAG;
  frame.can_dlc = 0;

  switch (ch) {
    case 1:
      nbytes = write(can_s1, &frame, sizeof(frame));
      break;
    default:
      nbytes = write(can_s0, &frame, sizeof(frame));
      break;
  }

  if (nbytes != sizeof(frame)) return 1;

  return 0;
}

void MotorControl::RecvMotorStatus() {
  while (is_inited) {
    // printf("Is running RecvMotorStatus\n");
    int nbytes_x = 0;
    nbytes_x = read(can_s0, &rsv_frame0, sizeof(rsv_frame0));
    if (nbytes_x > 0) {
      data_proc(0, &rsv_frame0);
    }

    nbytes_x = 0;
    nbytes_x = read(can_s1, &rsv_frame1, sizeof(rsv_frame1));
    if (nbytes_x > 0) {
      data_proc(1, &rsv_frame1);
    }
  }
}

int MotorControl::send_by_bus_block(unsigned char ch, unsigned short id, unsigned char* data, unsigned char len) {
  int ii;
  struct can_frame frame;
  memset(&frame, 0, sizeof(struct can_frame));
  frame.can_id = id;
  frame.can_dlc = len;
  for (ii = 0; ii < len; ++ii) frame.data[ii] = data[ii];
  switch (ch) {
    case 1:
      nbytes = write(can_s1, &frame, sizeof(frame));
      time_set_ = timer_.getSeconds();
#ifdef PRINT_CAN_SEND
      printf("Can bus 1 send command at time %f.\n", timer_.getMs());
#endif  // PRINT_CAN_SEND
      bus1_rsv();
      break;
    default:
      nbytes = write(can_s0, &frame, sizeof(frame));
      time_set_ = timer_.getSeconds();
#ifdef PRINT_CAN_SEND
      printf("Can bus 0 send command at time %f.\n", timer_.getMs());
#endif  // PRINT_CAN_SEND
      bus0_rsv();
      break;
  }

  if (nbytes != sizeof(frame)) return 1;

  return 0;
}

int MotorControl::send_by_bus_nonblock(unsigned char ch, unsigned short id, unsigned char* data, unsigned char len) {
  int ii;
  struct can_frame frame;
  memset(&frame, 0, sizeof(struct can_frame));
  frame.can_id = id;
  frame.can_dlc = len;
  for (ii = 0; ii < len; ++ii) frame.data[ii] = data[ii];

  int rsv_timeout = kCanRecvTimeout;
  switch (ch) {
    case 1:
      nbytes = write(can_s1, &frame, sizeof(frame));
      is_not_rsv1 = true;
      while (is_not_rsv1 && rsv_timeout > 0) {
        if (bus1_rsv() == 1) {
          is_not_rsv1 = false;
#ifdef PRINTTIMEOUT
          printf("Can bus1 receive at timeout %d.\n", kCanRecvTimeout - rsv_timeout);
#endif
          break;
        } else {
          --rsv_timeout;
        }
      }
      break;
    default:
      nbytes = write(can_s0, &frame, sizeof(frame));
      is_not_rsv0 = true;
      while (is_not_rsv0 && rsv_timeout > 0) {
        if (bus0_rsv() == 1) {
          is_not_rsv0 = false;
#ifdef PRINTTIMEOUT
          printf("Can bus0 receive at timeout %d.\n", kCanRecvTimeout - rsv_timeout);
#endif
          break;
        } else {
          --rsv_timeout;
        }
      }
      break;
  }

  if (nbytes != sizeof(frame)) return 1;

  return 0;
}

int MotorControl::send_by_bus0_timeout(unsigned short id, unsigned char* data, unsigned char len) {
  int ii;
  int rsv_timeout = kCanRecvTimeout;
  struct can_frame frame;
  memset(&frame, 0, sizeof(struct can_frame));
  frame.can_id = id;
  frame.can_dlc = len;
  for (ii = 0; ii < len; ++ii) frame.data[ii] = data[ii];

  nbytes = write(can_s0, &frame, sizeof(frame));

  is_not_rsv0 = true;
  while (is_not_rsv0 && rsv_timeout > 0) {
    if (bus0_rsv() == 1) {
      is_not_rsv0 = false;
      break;
    } else {
      --rsv_timeout;
    }
  }

  if (nbytes != sizeof(frame)) return 1;

  return 0;
}

int MotorControl::send_by_bus1_timeout(unsigned short id, unsigned char* data, unsigned char len) {
  int ii;
  struct can_frame frame;
  int rsv_timeout = kCanRecvTimeout;
  memset(&frame, 0, sizeof(struct can_frame));
  frame.can_id = id;
  frame.can_dlc = len;
  for (ii = 0; ii < len; ++ii) frame.data[ii] = data[ii];

  nbytes = write(can_s1, &frame, sizeof(frame));

  is_not_rsv1 = true;
  while (is_not_rsv1 && rsv_timeout > 0) {
    if (bus1_rsv() == 1) {
      is_not_rsv1 = false;
      break;
    } else {
      --rsv_timeout;
    }
  }

  if (nbytes != sizeof(frame)) return 1;

  return 0;
}

// 0: non-receive; 1:received
int MotorControl::bus0_rsv() {
  int nbytes0 = 0;
  if (is_inited) {
    nbytes0 = read(can_s0, &rsv_frame0, sizeof(rsv_frame0));
    if (nbytes0 > 0) {
      data_proc(0, &rsv_frame0);
      time_get_ = timer_.getSeconds();
      return 1;
    } else {
      return 0;
    }
  }
  return 0;
}

// 0: non-receive; 1:received
int MotorControl::bus1_rsv() {
  int nbytes1 = 0;
  if (is_inited) {
    nbytes1 = read(can_s1, &rsv_frame1, sizeof(rsv_frame1));
    if (nbytes1 > 0) {
      data_proc(1, &rsv_frame1);
      time_get_ = timer_.getSeconds();
      return 1;
    } else {
      return 0;
    }
  }
  return 0;
}

void MotorControl::bus_rsv() {
  int nbytes0 = 0;
  int nbytes1 = 0;

  if (is_inited) {
    nbytes0 = read(can_s0, &rsv_frame0, sizeof(rsv_frame0));

    if (nbytes0 > 0) data_proc(0, &rsv_frame0);
  }

  if (is_inited) {
    nbytes1 = read(can_s1, &rsv_frame1, sizeof(rsv_frame1));
    if (nbytes1 > 0) data_proc(1, &rsv_frame1);
  }
}

int MotorControl::de_init_bus() {
  close(can_s0);
  system("sudo ifconfig can0 down");

  close(can_s1);
  system("sudo ifconfig can1 down");
  return 0;
}

void MotorControl::data_proc(unsigned char ch, struct can_frame* frame) {
  unsigned short id = frame->can_id;
  unsigned short cmd = id >> 4;
  unsigned short tmpu16;
  short tmp16;
  float tmpfloat;
  int idx = 0;
  id &= 0x00F;
  if (id > 0 && id < 13) {
    idx = id - 1;
    motors[idx].ch = ch;
    motors[idx].rsv_cnt++;
    switch (cmd) {
      case 0x0C:
        tmpfloat = *(float*)frame->data;
        motors[idx].position_real =
            tmpfloat * motors[idx].direction * motors[idx].pos_ratio + motors[idx].position_zero;
        break;
      case 0x0D:
        tmpfloat = *(float*)frame->data;
        motors[idx].velocity_real = tmpfloat * motors[idx].direction * motors[idx].vel_ratio;
        break;
      case 0x0E:
        tmpfloat = *(float*)frame->data;
        motors[idx].torque_real = tmpfloat * motors[idx].direction * motors[idx].tau_ratio;
        break;
      case 0x0B:
      case 0x0A:
      case 0x09:
        if (8 == frame->can_dlc) {
          // current unit returned from driver is [A] - [Nm]
          tmpu16 = frame->data[0] + (unsigned short)frame->data[1] * 256;
          tmp16 = tmpu16;
          motors[idx].torque_real = (float)tmp16 * 0.001f * motors[idx].direction * motors[idx].tau_ratio;

          // speed unit returned from driver is [rps] - [rad/s]
          tmpu16 = frame->data[2] + (unsigned short)frame->data[3] * 256;
          tmp16 = tmpu16;
          motors[idx].velocity_real = (float)tmp16 * 0.01f * motors[idx].direction * motors[idx].vel_ratio;

          // position unit returned from driver is [rad] - [rad]
          tmpfloat = *(float*)(frame->data + 4);
          motors[idx].position_real =
              tmpfloat * motors[idx].direction * motors[idx].pos_ratio + motors[idx].position_zero;
        }
        break;
    }
  }
}

// set motor enable
// idx----motor id
// en----motor enable or not
void MotorControl::set_en(int idx, bool en) {
  unsigned short id = motors[idx].id;
  unsigned short cmd;
  if (en)
    cmd = 0x01;
  else
    cmd = 00;
  id += cmd << 4;
  send_by_bus(motors[idx].ch, id);
}

// set motor enable
// idx----motor id
// mode----motor var mode: 0--Current ,1--speed,2--position
void MotorControl::get_var(int idx, int mode) {
  unsigned short id = motors[idx].id;
  unsigned short cmd;
  switch (mode) {
    case 0:
      cmd = 0x0E;
      break;
    case 1:
      cmd = 0x0D;
      break;
    case 2:
      cmd = 0x0C;
      break;
  }
  id += cmd << 4;
  send_by_bus(motors[idx].ch, id);
}

// set motor valueS
// idx----motor id
// mode----motor control mode: 0--Current mode,1--speed mode,2--position mode
// value----motor control value
void MotorControl::set_value(int idx, int mode, float value) {
  unsigned char tx_buff[8] = {0};
  unsigned short id = motors[idx].id;
  unsigned short cmd = 0;
  float snd_value = 0;

  switch (mode) {
    case 0: {
      // Current mode, current = torque / tau_ratio
      cmd = 0x0B;
      snd_value = value * motors[idx].direction / motors[idx].tau_ratio;
      break;
    }
    case 1: {
      // Speed mode
      cmd = 0x0A;
      snd_value = value * motors[idx].direction / motors[idx].vel_ratio;
      break;
    }
    case 2: {
      // Position mode
      cmd = 0x09;
      snd_value = (value - motors[idx].position_zero) * motors[idx].direction / motors[idx].pos_ratio;
      break;
    }
  }

  unsigned char* pdata = (unsigned char*)&snd_value;
  tx_buff[0] = pdata[0];
  tx_buff[1] = pdata[1];
  tx_buff[2] = pdata[2];
  tx_buff[3] = pdata[3];

  id += cmd << 4;

#ifdef USENONBLOCK
#  ifdef SPLITSENDRECV
  send_by_bus(motors[idx].ch, id, tx_buff, 4);
#  else
  send_by_bus_nonblock(motors[idx].ch, id, tx_buff, 4);
#  endif
#else
#  ifdef SPLITSENDRECV
  send_by_bus(motors[idx].ch, id, tx_buff, 4);
#  else
  send_by_bus_block(motors[idx].ch, id, tx_buff, 4);
#  endif
#endif
}

// set motor work mode
// idx----motor id
// mode----motor cmd mode: GET  获取参数  SET 设定参数
// can_cmd----控制指令，定义见MOTOR_CMD
// value----指令对应值
// 例如设定0号电机电流环工作模式：
// motor_cmd(0,SET,CAN_CONFIG_CONTROL_MODE,CONTROL_MODE_CURRENT);
void MotorControl::motor_cmd(int idx, int mode, int can_cmd, float value) {
  unsigned char tx_buff[8] = {0};
  unsigned short id = motors[idx].id;
  unsigned short cmd = 0x11;
  switch (mode) {
    case GET:  // 读取
      cmd = 0x12;
      break;
    case SET:  // 设置
      cmd = 0x11;
      break;
  }

  switch (can_cmd) {
    case CAN_CONFIG_MOTOR_POLE_PAIRS:  // 电机极对数
      tx_buff[0] = 0x01;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_MOTOR_PHASE_RESISTANCE:  // 相电阻
      tx_buff[0] = 0x02;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_MOTOR_PHASE_INDUCTANCE:  // 相电感
      tx_buff[0] = 0x03;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_INERTIA:  // 转矩惯量
      tx_buff[0] = 0x04;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_ENCODER_DIR_REV:  // 编码器方向
      tx_buff[0] = 0x05;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_ENCODER_OFFSET:  // 编码器偏置值
      tx_buff[0] = 0x06;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_CALIB_VALID:  // 校准是否有效
      tx_buff[0] = 0x07;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_CONTROL_MODE:  // 控制模式
      tx_buff[0] = 0x0A;
      tx_buff[4] = (unsigned char)value;
      break;
    case CAN_CONFIG_CURRENT_RAMP_RATE:  // 电流爬坡速率
      tx_buff[0] = 0x0B;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_VEL_RAMP_RATE:  // 速度爬坡速率
      tx_buff[0] = 0x0C;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_TRAJ_VEL:  // 梯形速度
      tx_buff[0] = 0x0D;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_TRAJ_ACCEL:  // 梯形加速度
      tx_buff[0] = 0x0E;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_TRAJ_DECEL:  // 梯形减速度
      tx_buff[0] = 0x0F;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_POS_GAIN:  // 位置P增益
      tx_buff[0] = 0x10;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_VEL_GAIN:  // 速度P增益
      tx_buff[0] = 0x11;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_VEL_INTEGRATOR_GAIN:  // 速度积分增益
      tx_buff[0] = 0x12;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_VEL_LIMIT:  // 速度限制
      tx_buff[0] = 0x13;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_CURRENT_LIMIT:  // 电流限制
      tx_buff[0] = 0x14;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_CURRENT_CTRL_P_GAIN:  // 电流环P增益
      tx_buff[0] = 0x15;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_CURRENT_CTRL_I_GAIN:  // 电流环积分增益
      tx_buff[0] = 0x16;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_PROTECT_UNDER_VOLTAGE:  // 低压保护电压
      tx_buff[0] = 0x17;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_PROTECT_OVER_VOLTAGE:  // 过压保护电压
      tx_buff[0] = 0x18;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_PROTECT_OVER_SPEED:  // 超速保护
      tx_buff[0] = 0x19;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_CAN_ID:  // 设置CAN ID
      tx_buff[0] = 0x1A;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_CAN_TIMEOUT_MS:  // 设置总线超时时间
      tx_buff[0] = 0x1B;
      float_char(value, tx_buff + 4);
      break;
    case CAN_CONFIG_CAN_SYNC_TARGET_ENABLE:  // 目前给定是否同步给驱动器
      tx_buff[0] = 0x1C;
      float_char(value, tx_buff + 4);
      break;
  }
  id += cmd << 4;
  send_by_bus(motors[idx].ch, id, tx_buff, 8);
}

void MotorControl::float_char(float value, unsigned char* buff) {
  float snd_value = value;
  unsigned char* pdata = (unsigned char*)&snd_value;
  buff[0] = pdata[0];
  buff[1] = pdata[1];
  buff[2] = pdata[2];
  buff[3] = pdata[3];
}
