#ifndef UART_CAN_MOTOR_UART_H_
#define UART_CAN_MOTOR_UART_H_

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>

#include "include/timer.h"

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

#define DEG_RAD M_PI / 180
#define R_MIN_RAD_S M_PI / 30

struct exCanIdInfo {
  uint32_t id : 8;
  uint32_t data : 16;
  uint32_t mode : 5;
  uint32_t res : 3;
};

/* CAN 发送报文结构 gd32 */
typedef struct {
  uint32_t tx_sfid;   /*!< 标准帧ID */
  uint32_t tx_efid;   /*!< 扩展帧ID */
  uint8_t tx_ff;      /*!< 帧格式，标准帧或数据帧 */
  uint8_t tx_ft;      /*!< 帧类型，数据帧或遥控帧 */
  uint8_t tx_dlen;    /*!< 数据长度 */
  uint8_t tx_data[8]; /*!< 传输的数据 */
} can_trasnmit_message_struct;

/* CAN 接收报文结构 gd32 */
typedef struct {
  uint32_t rx_sfid;   /*!< 标准帧ID */
  uint32_t rx_efid;   /*!< 扩展帧ID */
  uint8_t rx_ff;      /*!< 帧格式，标准帧或数据帧 */
  uint8_t rx_ft;      /*!< 帧类型，数据帧或遥控帧 */
  uint8_t rx_dlen;    /*!< 数据长度 */
  uint8_t rx_data[8]; /*!< 接收的数据 */
  uint8_t rx_fi;      /*!< 过滤器编号 */
} can_receive_message_struct;

class MotorUart {
 public:
  MotorUart();
  ~MotorUart();

  int Init();

  void GetStatus();

  void SetCommand();

  void Close();

  void Enable(uint8_t id, uint16_t master_id);

  void Reset(uint8_t id, uint16_t master_id);

  void ControlMode(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd);

  void ModeChange(uint8_t id, uint16_t master_id);

  void MotorWrite(uint8_t id, uint16_t master_id);

 private:
  int can_;
  struct ifreq ifr_;
  struct sockaddr_can addr_;

  float ref_;
  uint16_t index_;
  uint8_t runmode_;

  bool first_run_;
  int can_id_first_;

  int FloatToUint(float x, float x_min, float x_max, int bits);

  int CanTxd(struct can_frame* frame);

  void CanRxd();

  void DataProcess(struct can_frame* frame);
};

#endif  // UART_CAN_MOTOR_UART_H_
