#include <SoftwareSerial.h>

#define MYPORT_RX D7
#define MYPORT_TX D6

EspSoftwareSerial::UART myPort;

unsigned long g_start_time = millis();

void setup() {
  pinMode(MYPORT_RX, INPUT);
  pinMode(MYPORT_TX, OUTPUT);
  Serial.begin(9600);
  // 初始化软件串口，baud,config,rxPin,txPin,invert
  myPort.begin(9600, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
}

void loop() {
  if (millis() >= g_start_time + 5000) {
    myPort.println("hello world");
    Serial.println("HELLO WORLD");
    // 更新g_start_time
    g_start_time = millis();
  }
  // 如果软件串口有数据可读，则转发到硬件串口
  if (myPort.available() > 0) {
    Serial.write(myPort.read());
  }
  // 如果硬件串口有数据可读，则转发到软件串口
  if (Serial.available() > 0) {
    myPort.write(Serial.read());
  }
}
