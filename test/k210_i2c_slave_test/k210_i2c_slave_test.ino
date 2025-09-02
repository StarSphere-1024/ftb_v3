#include <Wire.h>
#include "WonderK210_I2C_Slave.h"

#define K210_I2C_SCL_PIN 36
#define K210_I2C_SDA_PIN 37

TwoWire *k210Wire = new TwoWire(1);
WonderK210_I2C *k210 = new WonderK210_I2C(k210Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("K210 I2C Face Detection Example");
  k210->begin(K210_I2C_SDA_PIN, K210_I2C_SCL_PIN);
}

void loop() {
  
  if (k210->update_data()) {
    Find_Box_st faceBox;

    if (k210->recive_box(&faceBox, K210_FIND_FACE_YOLO)) {
      Serial.print("Face Detected! ");
      Serial.print("x: ");
      Serial.print(faceBox.x);
      Serial.print(", y: ");
      Serial.print(faceBox.y);
      Serial.print(", w: ");
      Serial.print(faceBox.w);
      Serial.print(", h: ");
      Serial.println(faceBox.h);
    }
  } else {
     Serial.println("No valid data received.");
  }
  
  delay(200);
}