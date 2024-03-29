#ifndef SPI_CAN_MOTOR_SPI_H_
#define SPI_CAN_MOTOR_SPI_H_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <thread>

#include "include/timer.h"

namespace {
const float kGearRatio = 8.f;
const float kDriverRatio = 2.f * 3.1415926f / kGearRatio;
const float kKneeRatio = 1.f / 0.72266f;
const float kTorqueCoefficient = 0.21f;  // Nm/A
const int kCanRecvTimeout = 5000;  // 非阻塞式通讯模式下，接收超时的次数（丢包严重则改大此数）
}  // namespace

class MotorSpi {
 public:
  MotorSpi();
  ~MotorSpi();

  int ret, nbytes;
  int can_s0, can_s1;
  struct sockaddr_can addr0, addr1;
  struct ifreq ifr0, ifr1;
  bool is_inited;
  struct can_frame rsv_frame0, rsv_frame1;
  bool is_not_rsv0;
  bool is_not_rsv1;
  Timer timer_;

  std::thread can_recv_thread_;

  void RecvMotorStatus();
  int init_bus();
  void init_recv_thread();
  int send_by_bus(unsigned char ch, unsigned short id, unsigned char* data, unsigned char len);
  int send_by_bus(unsigned char ch, unsigned short id);
  int send_by_bus_block(unsigned char ch, unsigned short id, unsigned char* data, unsigned char len);
  int send_by_bus_nonblock(unsigned char ch, unsigned short id, unsigned char* data, unsigned char len);
  int send_by_bus0_timeout(unsigned short id, unsigned char* data, unsigned char len);
  int send_by_bus1_timeout(unsigned short id, unsigned char* data, unsigned char len);
  void bus_rsv();
  int bus0_rsv();
  int bus1_rsv();
  int de_init_bus();  //
  void data_proc(unsigned char ch, struct can_frame* frame);

  struct MOTOR_VAR {
    unsigned short id;     // node id
    unsigned char ch;      // can cn
    unsigned char opmode;  // operate mode
    unsigned char statu;   // motor status
    int direction;         // 电机转向*模型方向
    float pos_ratio;       // 位置减速比
    float vel_ratio;       // 速度减速比
    float tau_ratio;       // 力矩减速比
    float position_set;    // 位置设定值
    float position_real;   // 位置真实值
    float position_zero;   // 位置零位
    float velocity_set;    // 速度设定值
    float velocity_real;   // 速度真实值
    float ke;              // 力矩系数 Nm/A
    float current_set;     // 电流设定值
    float current_real;    // 电流实际值
    int rsv_cnt;
  };

  enum MOTOR_MODE {
    CONTROL_MODE_CURRENT = 0,
    CONTROL_MODE_CURRENT_RAMP,
    CONTROL_MODE_VELOCITY,
    CONTROL_MODE_VELOCITY_RAMP,
    CONTROL_MODE_POSITION,
    CONTROL_MODE_POSITION_TRAP
  };

  enum MOTOR_CMD {
    CAN_CONFIG_MOTOR_POLE_PAIRS = 1,
    CAN_CONFIG_MOTOR_PHASE_RESISTANCE,
    CAN_CONFIG_MOTOR_PHASE_INDUCTANCE,
    CAN_CONFIG_INERTIA,
    CAN_CONFIG_ENCODER_DIR_REV,
    CAN_CONFIG_ENCODER_OFFSET,
    CAN_CONFIG_CALIB_VALID,
    CAN_CONFIG_CONTROL_MODE,
    CAN_CONFIG_CURRENT_RAMP_RATE,
    CAN_CONFIG_VEL_RAMP_RATE,
    CAN_CONFIG_TRAJ_VEL,
    CAN_CONFIG_TRAJ_ACCEL,
    CAN_CONFIG_TRAJ_DECEL,
    CAN_CONFIG_POS_GAIN,
    CAN_CONFIG_VEL_GAIN,
    CAN_CONFIG_VEL_INTEGRATOR_GAIN,
    CAN_CONFIG_VEL_LIMIT,
    CAN_CONFIG_CURRENT_LIMIT,
    CAN_CONFIG_CURRENT_CTRL_P_GAIN,
    CAN_CONFIG_CURRENT_CTRL_I_GAIN,
    CAN_CONFIG_PROTECT_UNDER_VOLTAGE,
    CAN_CONFIG_PROTECT_OVER_VOLTAGE,
    CAN_CONFIG_PROTECT_OVER_SPEED,
    CAN_CONFIG_CAN_ID,
    CAN_CONFIG_CAN_TIMEOUT_MS,
    CAN_CONFIG_CAN_SYNC_TARGET_ENABLE
  };

  enum OPERATE { GET, SET };

  MOTOR_VAR motors[12];
  void float_char(float value, unsigned char* buff);
  void set_en(int idx, bool en);
  void set_value(int idx, int mode, float value);
  void motor_cmd(int idx, int mode, int can_cmd, float value);
  void get_var(int idx, int mode);
  void send_loop();
};

#endif  // SPI_CAN_MOTOR_SPI_H_
