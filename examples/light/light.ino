#define LIGHT 7  // 定义光线传感器连接的引脚为7
void setup() {
  Serial.begin(9600);     // 设置串口通信速率为9600波特，用于后续将传感器数据打印到串行监视器上
  pinMode(LIGHT, INPUT);  // 将引脚7设置为输入模式，以读取光线传感器的模拟输入值
}
void loop() {
  int sensorValue = analogRead(LIGHT);  // 函数从引脚7读取模拟输入值，并将其存储在变量 sensorValue 中
  Serial.print("Light value:");
  Serial.println(sensorValue);  //  将读取到的传感器值打印到串行监视器上
  delay(100);
     // 每次读取间隔100ms
}

