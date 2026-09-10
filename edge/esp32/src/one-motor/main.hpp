#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>

// エンコーダピン定義
const int ENCODER_PIN_A = 18; // 黄色線 (D18)
const int ENCODER_PIN_B = 19; // 緑色線 (D19)

// パルスカウント用変数（割り込み内で更新するため volatile）
volatile long encoder_ticks = 0;
long last_ticks = 0;
unsigned long last_print_time = 0;

// A相割り込みハンドラ
void IRAM_ATTR handle_encoder_interrupt() {
  int a_val = digitalRead(ENCODER_PIN_A);
  int b_val = digitalRead(ENCODER_PIN_B);

  // A相の立ち上がり/立ち下がりとB相のレベルから回転方向を判定
  if (a_val == b_val) {
    encoder_ticks++; // 正転
  } else {
    encoder_ticks--; // 逆転
  }
}

void _setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {
    // シリアルポートの起動待機
  }

  // エンコーダピンを入力（プルアップ有効）に設定
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  // A相の状態変化（立ち上がり・立ち下がりの両方）で割り込み
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handle_encoder_interrupt, CHANGE);

  Serial.println("======================================");
  Serial.println("  ESP32 Encoder Test Ready!");
  Serial.println("  Hand-turn the motor shaft/wheel.");
  Serial.println("======================================");
}

void _loop() {
  unsigned long now = millis();

  // 100msごとに現在のパルス数と差分を表示
  if (now - last_print_time >= 100) {
    last_print_time = now;

    long current_ticks = encoder_ticks;
    long diff_ticks = current_ticks - last_ticks;
    last_ticks = current_ticks;

    Serial.print("Total Ticks: ");
    Serial.print(current_ticks);
    Serial.print(" | Speed (ticks/100ms): ");
    Serial.println(diff_ticks);
  }
}

#endif // MAIN_HPP
