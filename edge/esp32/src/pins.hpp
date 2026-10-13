#pragma once

// GPIO1(TXD0)・GPIO3(RXD0)はラズパイとの直接UART接続用に空けておくため割り当てない。
// GPIO6〜11(内蔵フラッシュ接続)、GPIO34〜39(入力専用)も使わない。

// モーター(TB6612FNG) 左: Motor A / 右: Motor B
const int PIN_PWMA = 13;
const int PIN_AIN1 = 14;
const int PIN_AIN2 = 12;

const int PIN_PWMB = 25;
const int PIN_BIN1 = 27;
const int PIN_BIN2 = 26;

// 車輪エンコーダ
const int PIN_LEFT_ENC_A  = 18;
const int PIN_LEFT_ENC_B  = 19;
const int PIN_RIGHT_ENC_A = 16;
const int PIN_RIGHT_ENC_B = 17;

// IMU(GY-521 / MPU6050) I2C
const int PIN_SDA = 21;
const int PIN_SCL = 22;
