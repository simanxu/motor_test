# motor_test

ratio = 1/8

pos_output = pos_motor * ratio

vel = (pos_{t} - pos_{t-1}) / dt

vel_output = (pos_output_{t} - pos_output{t-1}) / dt
           = (pos_motor_{t} * ratio - pos_motor{t-1} * ratio) / dt
           = ratio * (pos_motor_{t} - pos_motor{t-1}) / dt
           = ratio * vel_motor

cur_output = tau_output / ke
           = (kp * pos_output_error + kd * vel_output_error + tau_ff_output) / ke
           = (kp * pos_motor_error * ratio + kd * vel_motor_error * ratio + tau_ff_output) / ke
           = (kp * pos_motor_error * ratio + kd * vel_motor_error * ratio + tau_ff_motor * ratio) / ke
           = ratio * (kp * pos_motor_error + kd * vel_motor_error + tau_ff_motor) / ke
           = ratio * tau_motor / ke
