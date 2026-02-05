#include "motor_control.h"
#include <Arduino.h>

void motor_init() {
  // Right Motor Setup
  pinMode(right_cha, OUTPUT);
  pinMode(right_chb, OUTPUT);
  pinMode(right_pwm, OUTPUT);

  // Left Motor Setup
  pinMode(left_cha, OUTPUT);
  pinMode(left_chb, OUTPUT);
  pinMode(left_pwm, OUTPUT);
}

void right_motor_forward(uint pwm){
  digitalWrite(right_cha, HIGH);
  digitalWrite(right_chb, LOW);
  analogWrite(right_pwm, pwm);
}

void right_motor_backward(uint pwm){
  digitalWrite(right_cha, LOW);
  digitalWrite(right_chb, HIGH);
  analogWrite(right_pwm, pwm);
}

void left_motor_forward(uint pwm){
  digitalWrite(left_cha, LOW);
  digitalWrite(left_chb, HIGH);
  analogWrite(left_pwm, pwm);
}

void left_motor_backward(uint pwm){
  digitalWrite(left_cha, HIGH);
  digitalWrite(left_chb, LOW);
  analogWrite(left_pwm, pwm);
}

void left_motor_stop(){
  digitalWrite(left_cha, LOW);
  digitalWrite(left_chb, LOW);
  analogWrite(left_pwm, 0);
}

void right_motor_stop(){
  digitalWrite(left_cha, LOW);
  digitalWrite(left_chb, LOW);
  analogWrite(left_pwm, 0);
}