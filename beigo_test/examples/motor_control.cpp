#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <time.h>

#include "motor_control.h"

using namespace std;

void handler(int c);

Motor_Control m_c;

void handler() {
  // cout << "ok\n";
}

int main() {
  struct sigevent evp;
  struct itimerspec ts;
  timer_t timer;

  int ret;

  // cout << "this is a motor control demo\r\n";
  m_c.init_bus();

  evp.sigev_value.sival_ptr = &timer;
  evp.sigev_notify = SIGEV_SIGNAL;
  evp.sigev_signo = SIGUSR1;
  signal(evp.sigev_signo, handler);
  ret = timer_create(CLOCK_REALTIME, &evp, &timer);
  if (ret) {
    perror("timer_create");
  }
  ts.it_interval.tv_sec = 0;
  ts.it_interval.tv_nsec = 1000000;
  ts.it_value.tv_sec = 0;
  ts.it_value.tv_nsec = 1000000;
  ret = timer_settime(timer, 0, &ts, NULL);
  if (ret) {
    perror("timer_settime");
  }

  while (1) {
    // m_c.set_value(4,0,100);
  }
}

Motor_Control::Motor_Control() {
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

  for (int ii = 0; ii < 12; ii++) {
    motors[ii].position_zero = 0;
    motors[ii].position_k = 1;
    motors[ii].current_k = 1;
  }

  memset(&rsv_frame0, 0, sizeof(struct can_frame));
  memset(&rsv_frame1, 0, sizeof(struct can_frame));
  init_bus();
}
Motor_Control::~Motor_Control() { de_init_bus(); }
int Motor_Control::init_bus() {
  system("sudo ip link set can0 up type can bitrate 1000000");
  system("sudo ip link set can1 up type can bitrate 1000000");

  system("sudo ifconfig can0 txqueuelen 65536");
  system("sudo ifconfig can1 txqueuelen 65536");
  // 1.Create socket
  can_s0 = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW); //|SOCK_NONBLOCK
  if (can_s0 < 0) {
    return 1;
  }
  can_s1 = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW); //|SOCK_NONBLOCK
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
  ret = bind(can_s0, (struct sockaddr *)&addr0, sizeof(addr0));
  if (ret < 0) {
    return 1;
  }

  addr1.can_family = PF_CAN;
  addr1.can_ifindex = ifr1.ifr_ifindex;
  ret = bind(can_s1, (struct sockaddr *)&addr1, sizeof(addr1));
  if (ret < 0) {
    return 1;
  }

  // 4.Define receive rules
  // struct can_filter rfilter[1];
  // rfilter[0].can_id = 0x123;
  // rfilter[0].can_mask = CAN_SFF_MASK;
  // setsockopt(can_s0, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
  // setsockopt(can_s1, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  is_inited = true;
  return 0;
}

void Motor_Control::send_loop() {}

int Motor_Control::send_by_bus(unsigned char ch, unsigned short id,
                               unsigned char *data, unsigned char len) {
  int ii;
  struct can_frame frame;
  memset(&frame, 0, sizeof(struct can_frame));
  frame.can_id = id;
  frame.can_dlc = len;
  for (ii = 0; ii < len; ii++)
    frame.data[ii] = data[ii];
  switch (ch) {
  case 1:
    nbytes = write(can_s1, &frame, sizeof(frame));
    break;
  default:
    nbytes = write(can_s0, &frame, sizeof(frame));
    break;
  }

  if (nbytes != sizeof(frame))
    return 1;

  return 0;
}
int Motor_Control::send_by_bus(unsigned char ch, unsigned short id) {
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

  if (nbytes != sizeof(frame))
    return 1;

  return 0;
}
int Motor_Control::send_by_bus_block(unsigned char ch, unsigned short id,
                                     unsigned char *data, unsigned char len) {
  int ii;
  struct can_frame frame;
  memset(&frame, 0, sizeof(struct can_frame));
  frame.can_id = id;
  frame.can_dlc = len;
  for (ii = 0; ii < len; ii++)
    frame.data[ii] = data[ii];
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

  if (nbytes != sizeof(frame))
    return 1;

  return 0;
}

int Motor_Control::send_by_bus0_timeout(unsigned short id, unsigned char *data,
                                        unsigned char len) {
  int ii;
  int rsv_timeout = CAN_TIMEOUT;
  struct can_frame frame;
  memset(&frame, 0, sizeof(struct can_frame));
  frame.can_id = id;
  frame.can_dlc = len;
  for (ii = 0; ii < len; ii++)
    frame.data[ii] = data[ii];
  is_rsv0_l = true;
  nbytes = write(can_s0, &frame, sizeof(frame));
  while (is_rsv0_l && rsv_timeout > 0) {
    bus0_rsv();
    if (rsv_timeout > 0)
      rsv_timeout--;
  }

  if (nbytes != sizeof(frame))
    return 1;

  return 0;
}

int Motor_Control::send_by_bus1_timeout(unsigned short id, unsigned char *data,
                                        unsigned char len) {
  int ii;
  struct can_frame frame;
  int rsv_timeout = CAN_TIMEOUT;
  memset(&frame, 0, sizeof(struct can_frame));
  frame.can_id = id;
  frame.can_dlc = len;
  for (ii = 0; ii < len; ii++)
    frame.data[ii] = data[ii];

  is_rsv1_l = true;
  nbytes = write(can_s1, &frame, sizeof(frame));
  while (is_rsv1_l && rsv_timeout > 0) {
    bus1_rsv();
    if (rsv_timeout > 0)
      rsv_timeout--;
  }
  if (nbytes != sizeof(frame))
    return 1;

  return 0;
}

void Motor_Control::bus0_rsv() {
  int nbytes0 = 0;
  if (is_inited) {
    nbytes0 = read(can_s0, &rsv_frame0, sizeof(rsv_frame0));
    if (nbytes0 > 0) {
      data_proc(0, &rsv_frame0);
      is_rsv1_l = false;
    }
  }
}

void Motor_Control::bus1_rsv() {
  int nbytes1 = 0;
  if (is_inited) {
    nbytes1 = read(can_s1, &rsv_frame1, sizeof(rsv_frame1));
    if (nbytes1 > 0) {
      data_proc(1, &rsv_frame1);
      is_rsv0_l = false;
    }
  }
}

void Motor_Control::bus_rsv() {
  int nbytes0 = 0;
  int nbytes1 = 0;

  if (is_inited) {
    nbytes0 = read(can_s0, &rsv_frame0, sizeof(rsv_frame0));

    if (nbytes0 > 0)
      data_proc(0, &rsv_frame0);
  }

  if (is_inited) {
    nbytes1 = read(can_s1, &rsv_frame1, sizeof(rsv_frame1));
    if (nbytes1 > 0)
      data_proc(1, &rsv_frame1);
  }
}

int Motor_Control::de_init_bus() {
  close(can_s0);
  system("sudo ifconfig can0 down");

  close(can_s1);
  system("sudo ifconfig can1 down");
  return 0;
}

void Motor_Control::data_proc(unsigned char ch, struct can_frame *frame) {
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
    switch (cmd) {
    case 0x0C: // position value
      tmpfloat = *(float *)frame->data;
      motors[idx].position_real =
          tmpfloat * motors[idx].position_k - motors[idx].position_zero;
      break;
    case 0x0D: // speed value
      tmpfloat = *(float *)frame->data;
      motors[idx].velocity_real = tmpfloat;
      break;
    case 0x0E: // current value
      tmpfloat = *(float *)frame->data;
      motors[idx].current_real = tmpfloat * motors[idx].current_k;
      break;
    case 0x0B:
    case 0x0A:
    case 0x09:
      if (8 == frame->len) {
        tmpu16 = frame->data[0] + (unsigned short)frame->data[1] * 256;
        tmp16 = tmpu16;
        motors[idx].current_real = (float)tmp16 * 0.001f;

        tmpu16 = frame->data[3] + (unsigned short)frame->data[4] * 256;
        tmp16 = tmpu16;
        motors[idx].velocity_real = (float)tmp16 * 0.1f;

        tmpfloat = *(float *)(frame->data + 4);
        motors[idx].position_real =
            tmpfloat * motors[idx].position_k - motors[idx].position_zero;
      }
      break;
    }
  }
}

// set motor enable
// idx----motor id
// en----motor enable or not
void Motor_Control::set_en(int idx, bool en) {
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
void Motor_Control::get_var(int idx, int mode) {
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
void Motor_Control::set_value(int idx, int mode, float value) {
  unsigned char tx_buff[8] = {0};
  unsigned short id = motors[idx].id;
  unsigned short cmd = 0;
  float snd_value = 0;

  switch (mode) {
  case 0: // Current mode
    cmd = 0x0B;
    snd_value = value / motors[idx].current_k;
    break;
  case 1: // Speed mode
    cmd = 0x0A;
    snd_value = value;
    break;
  case 2: // Position mode
    cmd = 0x09;
    snd_value = (value + motors[idx].position_zero) / motors[idx].position_k;
    break;
  }
  unsigned char *pdata = (unsigned char *)&snd_value;
  tx_buff[0] = pdata[0];
  tx_buff[1] = pdata[1];
  tx_buff[2] = pdata[2];
  tx_buff[3] = pdata[3];

  id += cmd << 4;
  send_by_bus_block(motors[idx].ch, id, tx_buff, 4);
}

// set motor work mode
// idx----motor id
// mode----motor cmd mode: GET  获取参数  SET 设定参数
// can_cmd----控制指令，定义见MOTOR_CMD
// value----指令对应值
// 例如设定0号电机电流环工作模式：
// motor_cmd(0,SET,CAN_CONFIG_CONTROL_MODE,CONTROL_MODE_CURRENT);
void Motor_Control::motor_cmd(int idx, int mode, int can_cmd, float value) {
  unsigned char tx_buff[8] = {0};
  unsigned short id = motors[idx].id;
  unsigned short cmd = 0x11;
  switch (mode) {
  case GET: // 读取
    cmd = 0x12;
    break;
  case SET: // 设置
    cmd = 0x11;
    break;
  }

  switch (can_cmd) {
  case CAN_CONFIG_MOTOR_POLE_PAIRS: // 电机极对数
    tx_buff[0] = 0x01;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_MOTOR_PHASE_RESISTANCE: // 相电阻
    tx_buff[0] = 0x02;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_MOTOR_PHASE_INDUCTANCE: // 相电感
    tx_buff[0] = 0x03;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_INERTIA: // 转矩惯量
    tx_buff[0] = 0x04;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_ENCODER_DIR_REV: // 编码器方向
    tx_buff[0] = 0x05;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_ENCODER_OFFSET: // 编码器偏置值
    tx_buff[0] = 0x06;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_CALIB_VALID: // 校准是否有效
    tx_buff[0] = 0x07;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_CONTROL_MODE: // 控制模式
    tx_buff[0] = 0x0A;
    tx_buff[4] = (unsigned char)value;
    break;
  case CAN_CONFIG_CURRENT_RAMP_RATE: // 电流爬坡速率
    tx_buff[0] = 0x0B;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_VEL_RAMP_RATE: // 速度爬坡速率
    tx_buff[0] = 0x0C;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_TRAJ_VEL: // 梯形速度
    tx_buff[0] = 0x0D;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_TRAJ_ACCEL: // 梯形加速度
    tx_buff[0] = 0x0E;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_TRAJ_DECEL: // 梯形减速度
    tx_buff[0] = 0x0F;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_POS_GAIN: // 位置P增益
    tx_buff[0] = 0x10;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_VEL_GAIN: // 速度P增益
    tx_buff[0] = 0x11;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_VEL_INTEGRATOR_GAIN: // 速度积分增益
    tx_buff[0] = 0x12;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_VEL_LIMIT: // 速度限制
    tx_buff[0] = 0x13;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_CURRENT_LIMIT: // 电流限制
    tx_buff[0] = 0x14;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_CURRENT_CTRL_P_GAIN: // 电流环P增益
    tx_buff[0] = 0x15;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_CURRENT_CTRL_I_GAIN: // 电流环积分增益
    tx_buff[0] = 0x16;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_PROTECT_UNDER_VOLTAGE: // 低压保护电压
    tx_buff[0] = 0x17;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_PROTECT_OVER_VOLTAGE: // 过压保护电压
    tx_buff[0] = 0x18;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_PROTECT_OVER_SPEED: // 超速保护
    tx_buff[0] = 0x19;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_CAN_ID: // 设置CAN ID
    tx_buff[0] = 0x1A;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_CAN_TIMEOUT_MS: // 设置总线超时时间
    tx_buff[0] = 0x1B;
    float_char(value, tx_buff + 4);
    break;
  case CAN_CONFIG_CAN_SYNC_TARGET_ENABLE: // 目前给定是否同步给驱动器
    tx_buff[0] = 0x1C;
    float_char(value, tx_buff + 4);
    break;
  }
  id += cmd << 4;
  send_by_bus(motors[idx].ch, id, tx_buff, 8);
}

void Motor_Control::float_char(float value, unsigned char *buff) {
  float snd_value = value;
  unsigned char *pdata = (unsigned char *)&snd_value;
  buff[0] = pdata[0];
  buff[1] = pdata[1];
  buff[2] = pdata[2];
  buff[3] = pdata[3];
}
