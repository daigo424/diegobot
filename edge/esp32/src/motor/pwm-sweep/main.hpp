// 車輪を浮かせた状態で、PWMを段階的に変えたときの左右の車輪の速さを測る。
// シリアルで's'を受けてから回し始め、"duty,左m/s,右m/s,左の取りこぼし,右の取りこぼし"を1行ずつ出す(前進の向きを正)。
// 上げていく段と下げていく段の両方を測り、動き出す値と止まる値の差も見る。
#include <Arduino.h>

#include "../../pins.hpp"

const int PWM_CH_LEFT = 0;
const int PWM_CH_RIGHT = 1;
const float COUNTS_PER_REV = 2464.0f;
const float METERS_PER_COUNT = (0.068f * PI) / COUNTS_PER_REV;
const int HOLD_MS = 1200;
const int MEASURE_MS = 600;

const int8_t QEM[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
volatile long left_count = 0, right_count = 0;
volatile uint8_t left_state = 0, right_state = 0;
// 2本の信号が同時に変わった(=途中の変化を取りこぼした)回数
volatile long left_skips = 0, right_skips = 0;

void IRAM_ATTR on_left() {
  left_state = ((left_state << 2) | (digitalRead(PIN_LEFT_ENC_A) << 1) | digitalRead(PIN_LEFT_ENC_B)) & 0x0F;
  left_count += QEM[left_state];
  if (((left_state ^ (left_state >> 2)) & 0x3) == 0x3) left_skips++;
}

void IRAM_ATTR on_right() {
  right_state = ((right_state << 2) | (digitalRead(PIN_RIGHT_ENC_A) << 1) | digitalRead(PIN_RIGHT_ENC_B)) & 0x0F;
  right_count += QEM[right_state];
  if (((right_state ^ (right_state >> 2)) & 0x3) == 0x3) right_skips++;
}

void drive(int ch, int in1, int in2, int duty) {
  digitalWrite(in1, duty > 0 ? HIGH : LOW);
  digitalWrite(in2, duty < 0 ? HIGH : LOW);
  ledcWrite(ch, abs(duty));
}

// 右モーターは鏡像に付いているため、前進は逆回転。
void drive_both(int duty) {
  drive(PWM_CH_LEFT, PIN_AIN1, PIN_AIN2, duty);
  drive(PWM_CH_RIGHT, PIN_BIN1, PIN_BIN2, -duty);
}

void step(int duty) {
  drive_both(duty);
  delay(HOLD_MS - MEASURE_MS);
  noInterrupts();
  long l0 = left_count, r0 = right_count, ls0 = left_skips, rs0 = right_skips;
  interrupts();
  delay(MEASURE_MS);
  noInterrupts();
  long l1 = left_count, r1 = right_count, ls1 = left_skips, rs1 = right_skips;
  interrupts();
  float s = 1000.0f / MEASURE_MS;
  Serial.printf("%d,%.4f,%.4f,%ld,%ld\n", duty, (l1 - l0) * METERS_PER_COUNT * s, -(r1 - r0) * METERS_PER_COUNT * s,
                ls1 - ls0, rs1 - rs0);
}

void _setup() {
  for (int p : {PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2}) pinMode(p, OUTPUT);
  ledcSetup(PWM_CH_LEFT, 20000, 8);
  ledcAttachPin(PIN_PWMA, PWM_CH_LEFT);
  ledcSetup(PWM_CH_RIGHT, 20000, 8);
  ledcAttachPin(PIN_PWMB, PWM_CH_RIGHT);
  drive_both(0);
  for (int p : {PIN_LEFT_ENC_A, PIN_LEFT_ENC_B, PIN_RIGHT_ENC_A, PIN_RIGHT_ENC_B}) pinMode(p, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), on_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_B), on_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), on_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_B), on_right, CHANGE);
  Serial.begin(115200);
}

void _loop() {
  int c = Serial.read();
  // 'h'/'f': 中くらい(150)・最大(255)のdutyで10秒回し、回った量を出す(目で数えた回転数と比べる)。
  if (c == 'h' || c == 'f') {
    int duty = c == 'h' ? 150 : 255;
    noInterrupts();
    long l0 = left_count, r0 = right_count;
    interrupts();
    drive_both(duty);
    delay(10000);
    drive_both(0);
    delay(500);
    noInterrupts();
    long l1 = left_count, r1 = right_count;
    interrupts();
    Serial.printf("duty %d: 左 %.2f回転 右 %.2f回転\n", duty, (l1 - l0) / COUNTS_PER_REV, -(r1 - r0) / COUNTS_PER_REV);
    Serial.println("done");
    return;
  }
  if (c != 's') return;
  Serial.println("start");
  for (int sign : {1, -1}) {
    for (int d = 0; d <= 255; d += 15) step(sign * d);
    step(sign * 255);
    for (int d = 255; d >= 0; d -= 15) step(sign * d);
    drive_both(0);
    delay(1000);
  }
  drive_both(0);
  Serial.println("done");
}
