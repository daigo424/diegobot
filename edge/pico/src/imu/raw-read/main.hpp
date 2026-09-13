#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>
#include <Wire.h>

// GY-521(MPU6050)をライブラリ抜き・生のI2Cレジスタ読み書きだけで動かす。
// mpu6050-lib-readと同じ物理単位(m/s^2, rad/s)で出力し、結果を比較できるようにする。
const int PIN_SDA = 20;
const int PIN_SCL = 21;
const uint8_t MPU6050_ADDR = 0x68;

const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// デフォルトレンジ ±2g(AFS_SEL=0): 16384 LSB/g, ±250deg/s(FS_SEL=0): 131 LSB/(deg/s)
// (データシート「Register Map and Descriptions」記載の固定値)
const float ACCEL_SCALE = 16384.0f;
const float GYRO_SCALE  = 131.0f;
const float G_TO_MS2    = 9.80665f;
// DEG_TO_RADはArduinoコア(api/Common.h)で既に定義されているマクロなので、
// 名前を変えて衝突を避ける。

void write_register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void _setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();

  // 工場出荷時はSLEEPビットが立って止まっているため、0を書いて起こす。
  write_register(REG_PWR_MGMT_1, 0x00);
  // デフォルトレンジを明示的に書いておく(±2g, ±250deg/s)。
  write_register(REG_ACCEL_CONFIG, 0x00);
  write_register(REG_GYRO_CONFIG, 0x00);

  Serial.println("======================================");
  Serial.println("  MPU6050 raw register read ready");
  Serial.println("======================================");
}

void _loop() {
  // ACCEL_XOUT_Hから14バイト連続で読む(accel 6 + temp 2 + gyro 6)。
  // repeated start(endTransmission(false))を使い、読み出し中に他のI2C
  // トランザクションが割り込んで値が壊れるのを防ぐ。
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)14);

  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // TEMP_OUT_H/L (今回は使わない)
  int16_t raw_gx = (Wire.read() << 8) | Wire.read();
  int16_t raw_gy = (Wire.read() << 8) | Wire.read();
  int16_t raw_gz = (Wire.read() << 8) | Wire.read();

  float ax = (raw_ax / ACCEL_SCALE) * G_TO_MS2;
  float ay = (raw_ay / ACCEL_SCALE) * G_TO_MS2;
  float az = (raw_az / ACCEL_SCALE) * G_TO_MS2;
  float gx = (raw_gx / GYRO_SCALE) * DEG_TO_RAD;
  float gy = (raw_gy / GYRO_SCALE) * DEG_TO_RAD;
  float gz = (raw_gz / GYRO_SCALE) * DEG_TO_RAD;

  Serial.print("accel(m/s^2) x=");
  Serial.print(ax);
  Serial.print(" y=");
  Serial.print(ay);
  Serial.print(" z=");
  Serial.print(az);

  Serial.print("  gyro(rad/s) x=");
  Serial.print(gx);
  Serial.print(" y=");
  Serial.print(gy);
  Serial.print(" z=");
  Serial.println(gz);

  delay(200);
}

#endif // MAIN_HPP
