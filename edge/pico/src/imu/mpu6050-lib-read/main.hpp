#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// GY-521(MPU6050)からAdafruit_MPU6050ライブラリ経由で加速度・角速度を読む。
// フェーズ1のI2Cスキャン(0x68確認)は済んでいる前提。
const int PIN_SDA = 20;
const int PIN_SCL = 21;

Adafruit_MPU6050 mpu;

void _setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Check wiring.");
    while (true) delay(1000);
  }

  Serial.println("======================================");
  Serial.println("  MPU6050 found (Adafruit_MPU6050 library)");
  Serial.println("======================================");
}

void _loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("accel(m/s^2) x=");
  Serial.print(a.acceleration.x);
  Serial.print(" y=");
  Serial.print(a.acceleration.y);
  Serial.print(" z=");
  Serial.print(a.acceleration.z);

  Serial.print("  gyro(rad/s) x=");
  Serial.print(g.gyro.x);
  Serial.print(" y=");
  Serial.print(g.gyro.y);
  Serial.print(" z=");
  Serial.println(g.gyro.z);

  delay(200);
}

#endif // MAIN_HPP
