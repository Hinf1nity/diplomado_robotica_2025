#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// Type definition (common in Arduino/C environments)
typedef unsigned int uint;

const uint right_cha = 32;
const uint right_chb = 33;
const uint right_pwm = 25;

const uint left_cha = 13;
const uint left_chb = 26;
const uint left_pwm = 27;

void motor_init();
void right_motor_forward(uint pwm);
void right_motor_backward(uint pwm);
void left_motor_forward(uint pwm);
void left_motor_backward(uint pwm);
void right_motor_stop();
void left_motor_stop();

#endif // MOTOR_CONTROL_H