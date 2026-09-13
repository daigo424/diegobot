#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>

// --- ピン定義 ---
// 左輪（Motor 1）
const int LEFT_ENC_A = 18;
const int LEFT_ENC_B = 19;

// 右輪（Motor 2）
const int RIGHT_ENC_A = 16;
const int RIGHT_ENC_B = 17;

// --- パルスカウント変数 ---
volatile long left_ticks = 0;
volatile long right_ticks = 0;

long last_left_ticks = 0;
long last_right_ticks = 0;
unsigned long last_print_time = 0;

// --- 割り込みハンドラ (左輪) ---
void handle_left_encoder() {
  int a = digitalRead(LEFT_ENC_A);
  int b = digitalRead(LEFT_ENC_B);
  if (a == b) {
    left_ticks++;
  } else {
    left_ticks--;
  }
}

// --- 割り込みハンドラ (右輪) ---
void handle_right_encoder() {
  int a = digitalRead(RIGHT_ENC_A);
  int b = digitalRead(RIGHT_ENC_B);
  // ※左右でモーターの向きが反転するため、前進時に両方逆回転になるよう極性を合わせる
  if (a == b) {
    right_ticks--;
  } else {
    right_ticks++;
  }
}

void _setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  // ピン設定（内部プルアップ有効）
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  // 割り込み登録 (CHANGE: 立ち上がり/立ち下がり両方)
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), handle_left_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), handle_right_encoder, CHANGE);

  Serial.println("======================================");
  Serial.println("  Dual Wheel Encoder Test Ready!");
  Serial.println("======================================");
}

void _loop() {
  unsigned long now = millis();

  if (now - last_print_time >= 100) {
    last_print_time = now;

    long cur_l = left_ticks;
    long cur_r = right_ticks;
    long diff_l = cur_l - last_left_ticks;
    long diff_r = cur_r - last_right_ticks;
    last_left_ticks = cur_l;
    last_right_ticks = cur_r;

    Serial.print("Left: ");
    Serial.print(cur_l);
    Serial.print(" (");
    Serial.print(diff_l);
    Serial.print(") | Right: ");
    Serial.print(cur_r);
    Serial.print(" (");
    Serial.print(diff_r);
    Serial.println(")");
  }
}

#endif // MAIN_HPP
