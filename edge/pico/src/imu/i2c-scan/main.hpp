#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>
#include <Wire.h>

// GY-521(MPU6050)配線の疎通確認用。I2Cバスをスキャンして応答するアドレスを
// Serialへ出力するだけで、センサー値はまだ読まない。
const int PIN_SDA = 20;
const int PIN_SCL = 21;

void _setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();

  Serial.println("======================================");
  Serial.println("  I2C Scanner Ready! (MPU6050 wiring check)");
  Serial.println("======================================");
}

void _loop() {
  Serial.println("Scanning I2C bus...");
  int found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("  found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.println("  no device found");
  } else {
    Serial.print(found);
    Serial.println(" device(s) found. MPU6050 default address is 0x68 (0x69 if AD0=HIGH).");
  }

  delay(3000);
}

#endif // MAIN_HPP
