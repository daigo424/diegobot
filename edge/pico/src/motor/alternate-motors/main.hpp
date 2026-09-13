#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>

// production/main.hppと同じモーターピンをROS抜きで直接駆動し、
// ROS層とは無関係に配線・TB6612側の問題かを切り分ける。
// 左右を同時ではなく交互に動かすことで、TB6612の片チャンネルだけが
// 故障している場合でも、動く方だけ回るのを見て切り分けられるようにする。
const int PIN_PWMA = 4;
const int PIN_AIN1 = 2;
const int PIN_AIN2 = 3;

const int PIN_PWMB = 8;
const int PIN_BIN1 = 6;
const int PIN_BIN2 = 7;

const int PWM_FREQ  = 20000;
const int PWM_RANGE = 255;

void stop_motors() {
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
}

void _setup() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_RANGE);
  stop_motors();
}

void _loop() {
  // 左モーターだけ正転(duty 200/255) を2秒、停止を1秒
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, 200);
  delay(2000);
  stop_motors();
  delay(1000);

  // 右モーターだけ正転を2秒、停止を1秒
  digitalWrite(PIN_BIN1, HIGH);
  digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMB, 200);
  delay(2000);
  stop_motors();
  delay(1000);
}

#endif // MAIN_HPP
