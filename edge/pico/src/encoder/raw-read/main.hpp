#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>

// クアドラチャエンコーダをROS抜きでGPIO割り込みだけで動かし、カウント値・距離換算・
// 更新レートをSerialに出す(ステップB: 低速動作確認・高速時のパルス取りこぼし確認用)。
// ピン割当はdoc/wiring.md準拠。
const int PIN_LEFT_ENC_A  = 18;
const int PIN_LEFT_ENC_B  = 19;
const int PIN_RIGHT_ENC_A = 16;
const int PIN_RIGHT_ENC_B = 17;

// 車輪軸換算CPR(モーター軸11パルス/回転を4逓倍・56:1減速で換算)・車輪直径(68mm、実測)からの距離換算。
const float COUNTS_PER_REV = 2464.0f;
const float WHEEL_DIAMETER_M = 0.068f;
const float METERS_PER_COUNT = (WHEEL_DIAMETER_M * PI) / COUNTS_PER_REV;

// 4逓倍クアドラチャデコード用の状態遷移テーブル。
// インデックスは(前回のAB 2bit << 2 | 今回のAB 2bit)の4bit。
// 値は正しい遷移なら+1/-1、あり得ない遷移(パルス取りこぼし時に起き得る)なら0。
const int8_t QEM[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0,
};

volatile long left_count = 0;
volatile long right_count = 0;
volatile long left_invalid_transitions = 0;
volatile long right_invalid_transitions = 0;
volatile uint8_t left_state = 0;
volatile uint8_t right_state = 0;

void update_left_encoder() {
  left_state = ((left_state << 2) | (digitalRead(PIN_LEFT_ENC_A) << 1) | digitalRead(PIN_LEFT_ENC_B)) & 0x0F;
  int8_t delta = QEM[left_state];
  if (delta == 0) {
    left_invalid_transitions++;
  } else {
    left_count += delta;
  }
}

void update_right_encoder() {
  right_state = ((right_state << 2) | (digitalRead(PIN_RIGHT_ENC_A) << 1) | digitalRead(PIN_RIGHT_ENC_B)) & 0x0F;
  int8_t delta = QEM[right_state];
  if (delta == 0) {
    right_invalid_transitions++;
  } else {
    right_count += delta;
  }
}

void _setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), update_left_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_B), update_left_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), update_right_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_B), update_right_encoder, CHANGE);

  Serial.println("======================================");
  Serial.println("  Quadrature encoder raw read ready");
  Serial.println("======================================");
}

void _loop() {
  static long prev_left_count = 0;
  static long prev_right_count = 0;
  static unsigned long prev_ms = 0;

  unsigned long now_ms = millis();
  unsigned long elapsed_ms = now_ms - prev_ms;
  prev_ms = now_ms;

  // 割り込みで更新中の値を短時間だけ止めて一貫性のあるスナップショットを取る。
  noInterrupts();
  long l_count = left_count;
  long r_count = right_count;
  long l_invalid = left_invalid_transitions;
  long r_invalid = right_invalid_transitions;
  interrupts();

  long l_delta = l_count - prev_left_count;
  long r_delta = r_count - prev_right_count;
  prev_left_count = l_count;
  prev_right_count = r_count;

  float l_rate_cps = elapsed_ms > 0 ? (l_delta * 1000.0f / elapsed_ms) : 0;
  float r_rate_cps = elapsed_ms > 0 ? (r_delta * 1000.0f / elapsed_ms) : 0;

  Serial.print("L count=");
  Serial.print(l_count);
  Serial.print(" dist=");
  Serial.print(l_count * METERS_PER_COUNT, 4);
  Serial.print("m rate=");
  Serial.print(l_rate_cps, 1);
  Serial.print("cps invalid=");
  Serial.print(l_invalid);

  Serial.print("  |  R count=");
  Serial.print(r_count);
  Serial.print(" dist=");
  Serial.print(r_count * METERS_PER_COUNT, 4);
  Serial.print("m rate=");
  Serial.print(r_rate_cps, 1);
  Serial.print("cps invalid=");
  Serial.println(r_invalid);

  delay(200);
}

#endif // MAIN_HPP
