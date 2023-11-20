#include <iostream>

#include "uart_can/motor_uart.h"

int main() {
  std::cout << "start motor control test" << std::endl;
  MotorUart motor;
  int master_id = 0;  // user-defined master_id when enable motor
  int can_id = 1;
  motor.Init();

  motor.Enable(can_id, master_id);

  float torque = 0.2f;
  float MechPosition = 0.0;
  float speed = 1.f;
  float kp = 0.f;
  float kd = 1.f;
  for (float t = 0; t < 1.f; t += 0.01f) {
    motor.ControlMode(can_id, torque, MechPosition, speed, kp, kd);
    usleep(4000);
  }
  sleep(1);

  motor.Close();

  return 0;
}
