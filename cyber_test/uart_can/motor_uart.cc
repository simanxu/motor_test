#include "uart_can/motor_uart.h"

#include <iostream>

struct can_frame tx_msg;
struct can_frame rx_msg;
can_receive_message_struct rxMsg;
can_trasnmit_message_struct txMsg;

// 将扩展帧id解析为自定义数据结构
// #define txCanIdEx32 (((struct exCanIdInfo) & (txMsg.tx_efid)))
// #define rxCanIdEx32 (((struct exCanIdInfo) & (rxMsg.rx_efid)))
#define txCanIdEx (((struct exCanIdInfo*)&(tx_msg.can_id)))
#define rxCanIdEx (((struct exCanIdInfo*)&(rx_msg.can_id)))

namespace {
void print_can_frame(struct can_frame frame) {
  printf("CAN id (2进制): ");
  for (int i = 10; i >= 0; i--) {
    printf("%d", (frame.can_id >> i) & 1);
  }
  printf("\n");
  printf("CAN id (10进制): %d\n", frame.can_id);
  printf("CAN id (16进制): 0x%03X\n", frame.can_id);
  printf("CAN dlc (10进制): %d\n", frame.can_dlc);
  printf("CAN data (16进制):");
  for (int i = 0; i < frame.can_dlc; i++) {
    printf(" %02X", frame.data[i]);
  }
  printf("\n");
}
}  // namespace

MotorUart::MotorUart() {
  // tx_msg.can_id = CAN_EFF_FLAG | 0xff;
  tx_msg.can_id = 0xff;
  tx_msg.can_dlc = 8;
  memset(&tx_msg, 0, sizeof(struct can_frame));

  first_run_ = true;
}

MotorUart::~MotorUart() {}

int MotorUart::Init() {
  can_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
  if (can_ == -1) {
    perror("[USB] Open serial port failed");
    return (-1);
  } else {
    printf("[USB] Open uart /dev/ttyUSB0 success.\n");
  }

  if (isatty(STDIN_FILENO) == 0) {
    printf("[USB] Standard input is not a terminal device\n");
  } else {
    printf("[USB] Associate device with a terminal success.\n");
  }

  // // 1. create socket
  // #ifdef USENONBLOCK
  //   can_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  // #else
  //   can_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  // #endif
  //   if (can_ < 0) {
  //     perror("Error while opening socket");
  //     return -1;
  //   }

  //   // 2. specify can device
  //   // change this to your interface name
  //   const char* ifname = "can0";
  //   strcpy(ifr_.ifr_name, ifname);
  //   int ret = ioctl(can_, SIOCGIFINDEX, &ifr_);
  //   if (ret < 0) {
  //     perror("Error while configure the device");
  //     return -1;
  //   }

  //   // 3. bind the socket to can
  //   addr_.can_family = PF_CAN;
  //   addr_.can_ifindex = ifr_.ifr_ifindex;
  //   ret = bind(can_, (struct sockaddr*)&addr_, sizeof(addr_));
  //   if (ret < 0) {
  //     perror("Error while bind socket to device");
  //     return -1;
  //   }

  //   // 4. set socket as non-block
  //   ret = fcntl(can_, F_GETFL, 0);
  //   if (ret < 0) {
  //     perror("Failed to get socket flags");
  //     return -1;
  //   }
  //   ret |= O_NONBLOCK;
  //   if (fcntl(can_, F_SETFL, ret) < 0) {
  //     perror("Failed to set socket to non-blocking mode");
  //     return -1;
  //   }

  //   // 5. define receive rules
  //   //   struct can_filter rfilter[1];
  //   //   rfilter[0].can_id = 0x123;
  //   //   rfilter[0].can_mask = CAN_SFF_MASK;
  //   //   setsockopt(can_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  return 0;
}

void MotorUart::Close() { close(can_); }

void MotorUart::Reset(uint8_t id, uint16_t master_id) {
  txCanIdEx->mode = 4;
  txCanIdEx->id = id;
  txCanIdEx->res = 0;
  txCanIdEx->data = master_id;
  tx_msg.can_dlc = 8;
  for (uint8_t i = 0; i < 8; ++i) {
    tx_msg.data[i] = 0;
  }

  CanTxd(&tx_msg);
}

void MotorUart::Enable(uint8_t id, uint16_t master_id) {
  txCanIdEx->mode = 3;
  txCanIdEx->id = 1;
  txCanIdEx->res = 0;
  txCanIdEx->data = 0;
  tx_msg.can_dlc = 8;
  txCanIdEx->data = 0;

  CanTxd(&tx_msg);
}

int MotorUart::CanTxd(struct can_frame* frame) {
  tx_msg.can_id = frame->can_id;
  tx_msg.can_dlc = frame->can_dlc;
  for (int i = 0; i < 8; ++i) {
    tx_msg.data[i] = frame->data[i];
  }
  print_can_frame(tx_msg);

  int nbytes = write(can_, &tx_msg, sizeof(struct can_frame));
  return nbytes;
}

// if (first_run_) {
//   can_id_first_ = frame->can_id;
//   first_run_ = false;
// } else {
//   frame->can_id = can_id_first_;
// }
void MotorUart::CanRxd() {
  int nbytes = read(can_, &rx_msg, sizeof(struct can_frame));
  if (nbytes > 0) {
    DataProcess(&rx_msg);
  }
}

void MotorUart::DataProcess(struct can_frame* frame) {
  //   unsigned short id = frame->can_id;
  //   unsigned short cmd = id >> 4;
  //   unsigned short tmpu16;
  //   short tmp16;
  //   float tmpfloat;
  //   int idx = 0;
  //   id &= 0x00F;
  //   if (id > 0 && id < 13) {
  //     idx = id - 1;
  //     motors[idx].ch = ch;
  //     motors[idx].rsv_cnt++;
  //     switch (cmd) {
  //     case 0x0C: // position value
  //       tmpfloat = *(float *)frame->data;
  //       motors[idx].position_real =
  //           tmpfloat * motors[idx].pos_ratio - motors[idx].position_zero;
  //       break;
  //     case 0x0D: // speed value
  //       tmpfloat = *(float *)frame->data;
  //       motors[idx].velocity_real = tmpfloat;
  //       break;
  //     case 0x0E: // current value
  //       tmpfloat = *(float *)frame->data;
  //       motors[idx].current_real = tmpfloat * motors[idx].ke;
  //       break;
  //     case 0x0B:
  //     case 0x0A:
  //     case 0x09:
  //       if (8 == frame->can_dlc) {
  //         tmpu16 = frame->data[0] + (unsigned short)frame->data[1] * 256;
  //         tmp16 = tmpu16;
  //         motors[idx].current_real = (float)tmp16 * 0.001f *
  //                                    motors[idx].direction * motors[idx].ke *
  //                                    motors[idx].tau_ratio;

  //         tmpu16 = frame->data[2] + (unsigned short)frame->data[3] * 256;
  //         tmp16 = tmpu16;
  //         motors[idx].velocity_real =
  //             (float)tmp16 * 0.1f * motors[idx].direction *
  //             motors[idx].vel_ratio;
  //         // motors[idx].velocity_real *= 0.1;

  //         tmpfloat = *(float *)(frame->data + 4);
  //         motors[idx].position_real =
  //             motors[idx].direction * tmpfloat * motors[idx].pos_ratio +
  //             motors[idx].position_zero;
  //       }
  //       break;
  //     }
  //   }
}

void MotorUart::ControlMode(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd) {
  txCanIdEx->mode = 1;
  txCanIdEx->id = id;
  txCanIdEx->res = 0;
  txCanIdEx->data = FloatToUint(torque, T_MIN, T_MAX, 16);
  tx_msg.can_dlc = 8;
  tx_msg.data[0] = FloatToUint(MechPosition, P_MIN, P_MAX, 16) >> 8;
  tx_msg.data[1] = FloatToUint(MechPosition, P_MIN, P_MAX, 16);
  tx_msg.data[2] = FloatToUint(speed, V_MIN, V_MAX, 16) >> 8;
  tx_msg.data[3] = FloatToUint(speed, V_MIN, V_MAX, 16);
  tx_msg.data[4] = FloatToUint(kp, KP_MIN, KP_MAX, 16) >> 8;
  tx_msg.data[5] = FloatToUint(kp, KP_MIN, KP_MAX, 16);
  tx_msg.data[6] = FloatToUint(kd, KD_MIN, KD_MAX, 16) >> 8;
  tx_msg.data[7] = FloatToUint(kd, KD_MIN, KD_MAX, 16);
  CanTxd(&tx_msg);
}

void MotorUart::ModeChange(uint8_t id, uint16_t master_id) {
  txCanIdEx->mode = 0x12;
  txCanIdEx->id = id;
  txCanIdEx->res = 0;
  txCanIdEx->data = master_id;
  tx_msg.can_dlc = 8;
  for (uint8_t i = 0; i < 8; i++) {
    tx_msg.data[i] = 0;
  }
  memcpy(&tx_msg.data[0], &index_, sizeof(index_));
  memcpy(&tx_msg.data[4], &runmode_, sizeof(runmode_));
  CanTxd(&tx_msg);
}

void MotorUart::MotorWrite(uint8_t id, uint16_t master_id) {
  txCanIdEx->mode = 0x12;
  txCanIdEx->id = id;
  txCanIdEx->res = 0;
  txCanIdEx->data = master_id;
  tx_msg.can_dlc = 8;
  for (uint8_t i = 0; i < 8; i++) {
    tx_msg.data[i] = 0;
  }
  memcpy(&tx_msg.data[0], &index_, sizeof(index_));
  memcpy(&tx_msg.data[4], &ref_, sizeof(ref_));
  CanTxd(&tx_msg);
}

int MotorUart::FloatToUint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if (x > x_max)
    x = x_max;
  else if (x < x_min)
    x = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
