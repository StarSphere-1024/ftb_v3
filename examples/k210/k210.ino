#include <Arduino.h>
#include <Wire.h>
#include <WonderK210_I2C_Slave.h>

// K210相关硬件定义
#define K210_I2C_SCL_PIN 36
#define K210_I2C_SDA_PIN 37

TwoWire *k210Wire = new TwoWire(1);
WonderK210_I2C *k210 = new WonderK210_I2C(k210Wire);
Find_Box_st *result;

void setup() {
  Serial.begin(9600);
  k210->begin(K210_I2C_SDA_PIN, K210_I2C_SCL_PIN);
  result = new Find_Box_st();
}


void loop() {
  k210->update_data();
  if (k210->recive_box(result, K210_FIND_FACE_YOLO)) {
    Serial.print("x: ");
    Serial.println(result->x);
    Serial.print("y: ");
    Serial.println(result->y);
    Serial.print("w: ");
    Serial.println(result->w);
    Serial.print("h: ");
    Serial.println(result->h);
  } else {
    Serial.println("未收到人脸识别数据");
  }
  delay(100);
}
