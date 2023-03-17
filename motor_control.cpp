#include "motor_control.h"

#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <csignal>
#include <fstream>
#include <iostream>

// #define POSTEST      // 位置环'开环'测试
// #define CURRTEST     // 电流环'开环'测试
#define TRACKTEST  // 电流环'闭环'测试
// #define SINWAVE    // 开启后发送正弦波信号，关闭后电流环发送±1A的方波信号
// #define USENONBLOCK  // 开启后使用非阻塞式通讯，关闭后使用阻塞式通讯
// #define PRINTTIMEOUT  // 开启后打印多少个Timeout接收到信号
// #define SPLITSENDRECV  // 将接收与发送分离开

namespace {
const int kFrequency = 500;        // 控制频率(Hz)，数值允许1~500
const int kMotorCount = 12;        // 电机总数
const bool kSaveData = true;       // 是否保存测试数据
const float kKp = 8.0;             // 伺服跟踪Kp
const float kKd = 0.3;             // 伺服跟踪Kd
const int kCanRecvTimeout = 5000;  // 非阻塞式通讯模式下，接收超时的次数（丢包严重则改大此数）

const double kSquareCycle = 2000;  // 方波周期
const float kAmplitude = 2.0;      // sin幅值
const float kSinCycle = 2.0;       // sin周期
const float kSinOmega = 2 * M_PI / kSinCycle;
const int kMotorPrint = 0;

const float kGearRatio = 8.f;
const float kDriverRatio = 2.f * 3.14159265358979323846 / kGearRatio;
const float kKneeRatio = 1.f / 0.72266f;
const float kTorqueCoefficient = 0.21f;  // Nm/A

volatile std::sig_atomic_t stop_signal;
}  // namespace

MotorControl m_c;

timeval diff(timeval start, timeval end) {
  timeval temp;
  if ((end.tv_usec - start.tv_usec) < 0) {
    temp.tv_sec = end.tv_sec - start.tv_sec - 1;
    temp.tv_usec = 1000000 + end.tv_usec - start.tv_usec;
  } else {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_usec = end.tv_usec - start.tv_usec;
  }
  return temp;
}
double tos(timeval start) {
  double temp;
  temp = start.tv_usec * 0.000001;
  temp += (double)start.tv_sec;
  return temp;
}

int main() {
  std::signal(SIGINT, [](int ctrl_c) { stop_signal = ctrl_c; });
  std::ofstream log;
  log.open("../data.txt");

  struct timeval count_t0, count_t1;
  struct timezone tz;
  float set_pos[12] = {0};
  float set_vel[12] = {0};
  float set_cur[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  int send_iterval = 500 / kFrequency;
  if (kFrequency == 2) {
    send_iterval = 1000;
  }
  unsigned int sleep_count = 1800;
  int last_t = 0;
  int curr_t = 0;
  int iteration = 0;
  int enable_send = 1;
  double time_now = 0.0;

  m_c.init_bus();
  // enable motor
  for (int i = 0; i < kMotorCount; ++i) {
    std::cout << "id: " << i << " enabled" << std::endl;
    m_c.set_en(i, true);
  }
  std::cout << "CAN communication is setup successfully." << std::endl
            << "WARNING: Make sure the motor is enabled." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

#ifdef SPLITSENDRECV
  m_c.init_recv_thread();
#endif

#ifndef POSTEST
  float fai_bias[12] = {0};
  for (int n = 0; n < 10; ++n) {
    for (int ii = 0; ii < kMotorCount; ++ii) {
      m_c.set_value(ii, 0, 0.f);
    }
    usleep(5000);
  }
  for (int ii = 0; ii < kMotorCount; ++ii) {
    fai_bias[ii] = std::asin(m_c.motors[ii].position_real);
  }
#endif

  gettimeofday(&count_t1, &tz);
  Timer timer;
  timer.start();
  while (true) {
    time_now = timer.getSeconds();
    enable_send = iteration % send_iterval;

#ifdef SINWAVE
    gettimeofday(&count_t0, &tz);
    for (int ii = 0; ii < kMotorCount; ++ii) {
      set_pos[ii] = kAmplitude * std::sin(kSinOmega * time_now - fai_bias[ii]);
      set_vel[ii] = kAmplitude * kSinOmega * std::cos(kSinOmega * time_now - fai_bias[ii]);
      set_cur[ii] = 0;
    }
    // std::cout << "cmd=sin(w*t),t=" << time_now << ",cmd=" << set_value << std::endl;
#else
    curr_t = std::fmod(time_now, kSquareCycle);
    if (curr_t < last_t) {
      for (int ii = 0; ii < kMotorCount; ++ii) {
        set_cur[ii] *= -1;
      }
    }
    last_t = curr_t;
#endif

#ifdef POSTEST
    if (enable_send == 0) {
      for (int ii = 0; ii < kMotorCount; ++ii) {
        m_c.set_value(ii, 2, set_pos[ii]);
      }
      printf("Now %6.4f, SetPos %2.6f rad, ReadPos %2.6f rad, ReadVel %2.6f, ReadCurr %2.6f\n", time_now,
             set_pos[kMotorPrint], m_c.motors[kMotorPrint].position_real, m_c.motors[kMotorPrint].velocity_real,
             m_c.motors[kMotorPrint].current_real);
    }
#endif
#ifdef CURRTEST
    if (enable_send == 0) {
      for (int ii = 0; ii < kMotorCount; ++ii) {
        set_cur[ii] = set_pos[ii];
        m_c.set_value(ii, 0, set_cur[ii]);
      }
      printf("Now %6.4f, SetCurr %2.6f A, ReadCurr %2.6f A, ReadPos %2.6f, ReadVel %2.6f\n", time_now,
             set_pos[kMotorPrint], m_c.motors[kMotorPrint].current_real, m_c.motors[kMotorPrint].position_real,
             m_c.motors[kMotorPrint].velocity_real);
    }
#endif
#ifdef TRACKTEST
    if (enable_send == 0) {
      for (int ii = 0; ii < kMotorCount; ++ii) {
        // set_cur[ii] = kKp * (set_pos[ii] - m_c.motors[ii].position_real) +
        //               kKd * (set_vel[ii] - 0.5 * m_c.motors[ii].velocity_real);
        float value = set_cur[ii] * 0.2;  // 0.2表示0.2A
        m_c.set_value(ii, 0, value);
      }
      printf("Now %6.4f, SetPos %2.6f, ReadPos %2.6f, SetVel %2.6f, ReadVel %2.6f, SetCurr %2.6f, ReadCurr %2.6f\n",
             time_now, set_pos[kMotorPrint], m_c.motors[kMotorPrint].position_real, set_vel[kMotorPrint],
             m_c.motors[kMotorPrint].velocity_real, set_cur[kMotorPrint], m_c.motors[kMotorPrint].current_real);
    }
#endif

    if (kSaveData) {
      log << time_now << " ";
      for (int ii = 0; ii < kMotorCount; ++ii) {
        log << set_pos[ii] << " " << m_c.motors[ii].position_real << " " << set_vel[ii] << " "
            << m_c.motors[ii].velocity_real << " " << set_cur[ii] << " " << m_c.motors[ii].current_real << " ";
      }
      log << "\n";
    }

    iteration++;
    usleep(sleep_count);

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
  }
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
    motors[i].pos_ratio = kDriverRatio;
    motors[i].vel_ratio = kDriverRatio;
    motors[i].tau_ratio = kGearRatio;
    motors[i].position_zero = 0.f;
    motors[i].position_set = 0.f;
    motors[i].velocity_set = 0.f;
    motors[i].current_set = 0.f;
    motors[i].ke = kTorqueCoefficient;
  }

  // define motor position zero
  // motors[0].direction = -1;
  // motors[0].position_zero = 0.4227335;

  // motors[1].direction = -1;
  // motors[1].position_zero = -2.25485;

  // motors[2].pos_ratio = kDriverRatio / kKneeRatio;
  // motors[2].vel_ratio = kDriverRatio / kKneeRatio;
  // motors[2].tau_ratio = kGearRatio * kKneeRatio;
  // motors[2].direction = -1;
  // motors[2].position_zero = 2.468752;

  // motors[3].direction = -1;
  // motors[3].position_zero = -0.053213;

  // motors[4].direction = 1;
  // motors[4].position_zero = -2.86373;

  // motors[5].pos_ratio = kDriverRatio / kKneeRatio;
  // motors[5].vel_ratio = kDriverRatio / kKneeRatio;
  // motors[5].tau_ratio = kGearRatio * kKneeRatio;
  // motors[5].direction = 1;
  // motors[5].position_zero = 2.486871;

  // motors[6].direction = 1;
  // motors[6].position_zero = 0.039167;

  // motors[7].direction = -1;
  // motors[7].position_zero = -2.45351;

  // motors[8].pos_ratio = kDriverRatio / kKneeRatio;
  // motors[8].vel_ratio = kDriverRatio / kKneeRatio;
  // motors[8].tau_ratio = kGearRatio * kKneeRatio;
  // motors[8].direction = -1;
  // motors[8].position_zero = 2.5624647;

  // motors[9].direction = 1;
  // motors[9].position_zero = -0.512834;

  // motors[10].direction = 1;
  // motors[10].position_zero = -2.4107;

  // motors[11].pos_ratio = kDriverRatio / kKneeRatio;
  // motors[11].vel_ratio = kDriverRatio / kKneeRatio;
  // motors[11].tau_ratio = kGearRatio * kKneeRatio;
  // motors[11].direction = 1;
  // motors[11].position_zero = 2.876895;

  memset(&rsv_frame0, 0, sizeof(struct can_frame));
  memset(&rsv_frame1, 0, sizeof(struct can_frame));
  init_bus();
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
      bus1_rsv();
      break;
    default:
      nbytes = write(can_s0, &frame, sizeof(frame));
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
      case 0x0C:  // position value
        tmpfloat = *(float*)frame->data;
        motors[idx].position_real = tmpfloat * motors[idx].pos_ratio - motors[idx].position_zero;
        break;
      case 0x0D:  // speed value
        tmpfloat = *(float*)frame->data;
        motors[idx].velocity_real = tmpfloat;
        break;
      case 0x0E:  // current value
        tmpfloat = *(float*)frame->data;
        motors[idx].current_real = tmpfloat * motors[idx].ke;
        break;
      case 0x0B:
      case 0x0A:
      case 0x09:
        if (8 == frame->can_dlc) {
          tmpu16 = frame->data[0] + (unsigned short)frame->data[1] * 256;
          tmp16 = tmpu16;
          motors[idx].current_real =
              (float)tmp16 * 0.001f * motors[idx].direction * motors[idx].ke * motors[idx].tau_ratio;

          tmpu16 = frame->data[2] + (unsigned short)frame->data[3] * 256;
          tmp16 = tmpu16;
          motors[idx].velocity_real = (float)tmp16 * 0.1f * motors[idx].direction * motors[idx].vel_ratio;
          // motors[idx].velocity_real *= 0.1;

          tmpfloat = *(float*)(frame->data + 4);
          motors[idx].position_real =
              motors[idx].direction * tmpfloat * motors[idx].pos_ratio + motors[idx].position_zero;
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
    case 0:  // Current mode
      cmd = 0x0B;
      //   snd_value = value / motors[idx].ke;
      snd_value = value * motors[idx].direction / motors[idx].tau_ratio / motors[idx].ke;
      // if (id == 0) {
      //   printf("%f %f %f\n", motors[idx].position_real, value, motors[idx].current_real);
      //   // printf("%f %f\n", send_value, motors[idx].current_real);
      //   // printf("Can bus %d, send_tau: %f, send_curr: %f, real_curr: %f\n", id, value, send_value,
      //   //        motors[idx].current_real);
      // }
      break;
    case 1:  // Speed mode
      cmd = 0x0A;
      //   snd_value = value;
      snd_value = value * motors[idx].direction / motors[idx].vel_ratio;
      break;
    case 2:  // Position mode
      cmd = 0x09;
      //   snd_value = (value + motors[idx].position_zero) / motors[idx].pos_ratio;
      snd_value = (value - motors[idx].position_zero) / motors[idx].direction / motors[idx].pos_ratio;
      break;
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
