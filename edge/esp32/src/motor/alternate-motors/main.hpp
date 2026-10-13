#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>
#include "../../pins.hpp"

// production/main.hppと同じモーターピンをROS抜きで直接駆動し、ROS層とは無関係に
// 配線・電源・TB6612側の問題かを切り分ける。左右を交互に動かし、エンコーダから
// 実際の車輪速度[m/s]をSerialへ出力する。duty=255(全開)で十分な速度が出るなら
// ハードは正常で、問題はROS経由の指令側にある。
const int PWM_CH_LEFT  = 0;
const int PWM_CH_RIGHT = 1;
const int PWM_FREQ     = 20000;
const int PWM_RES      = 8;

// 車輪軸換算CPR(モーター軸11パルス/回転を4逓倍・56:1減速)と車輪直径(68mm)から距離へ換算。
const float COUNTS_PER_REV = 2464.0f;
const float WHEEL_DIAMETER_M = 0.068f;
const float METERS_PER_COUNT = (WHEEL_DIAMETER_M * PI) / COUNTS_PER_REV;

const int8_t QEM[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0,
};

volatile long left_count = 0;
volatile long right_count = 0;
volatile uint8_t left_state = 0;
volatile uint8_t right_state = 0;

void IRAM_ATTR update_left_encoder() {
  left_state = ((left_state << 2) | (digitalRead(PIN_LEFT_ENC_A) << 1) | digitalRead(PIN_LEFT_ENC_B)) & 0x0F;
  left_count += QEM[left_state];
}

void IRAM_ATTR update_right_encoder() {
  right_state = ((right_state << 2) | (digitalRead(PIN_RIGHT_ENC_A) << 1) | digitalRead(PIN_RIGHT_ENC_B)) & 0x0F;
  right_count += QEM[right_state];
}

void stop_motors() {
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);
  ledcWrite(PWM_CH_LEFT, 0);
  ledcWrite(PWM_CH_RIGHT, 0);
}

// 片輪を2秒回し、立ち上がり後の1秒間のエンコーダ増分から速度を出して表示する。
void run_and_measure(const char *name, int pwm_ch, int pin_in1, int pin_in2,
                     volatile long &count, int duty) {
  digitalWrite(pin_in1, HIGH);
  digitalWrite(pin_in2, LOW);
  ledcWrite(pwm_ch, duty);
  delay(1000);

  long start = count;
  delay(1000);
  long delta = count - start;

  stop_motors();

  float speed = abs(delta) * METERS_PER_COUNT; // 1秒間なのでそのままm/s
  Serial.print(name);
  Serial.print(" duty=");
  Serial.print(duty);
  Serial.print("/255  counts/s=");
  Serial.print(abs(delta));
  Serial.print("  speed=");
  Serial.print(speed, 3);
  Serial.println(" m/s");
}

void _setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  ledcSetup(PWM_CH_LEFT, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWMA, PWM_CH_LEFT);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWMB, PWM_CH_RIGHT);
  stop_motors();

  pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), update_left_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_B), update_left_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), update_right_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_B), update_right_encoder, CHANGE);

  Serial.println("======================================");
  Serial.println("  Motor full-power speed test (no ROS)");
  Serial.println("======================================");
}

void _loop() {
  run_and_measure("LEFT ", PWM_CH_LEFT, PIN_AIN1, PIN_AIN2, left_count, 255);
  delay(1000);
  run_and_measure("RIGHT", PWM_CH_RIGHT, PIN_BIN1, PIN_BIN2, right_count, 255);
  delay(1000);
}

#endif // MAIN_HPP
