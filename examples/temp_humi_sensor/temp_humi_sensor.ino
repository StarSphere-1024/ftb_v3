// 引入 Arduino 和 DHT 传感器库
#include <Arduino.h>
#include "DHT.h"

// DHT11 传感器引脚和类型定义
#define DHT_PIN 26         // DHT11 数据引脚
#define DHTTYPE DHT11      // 传感器类型
DHT dht(DHT_PIN, DHTTYPE); // 创建 DHT 对象

// 初始化串口和 DHT11 传感器
void setup()
{
    Serial.begin(9600);      // 初始化串口
    dht.begin();             // 初始化 DHT11
    Serial.println("DHT11 sensor setup complete.");
}

// 主循环，周期性读取并打印温湿度
void loop()
{
    Serial.print("Humidity: ");
    Serial.print(dht.readHumidity());      // 读取湿度
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(dht.readTemperature());   // 读取温度
    Serial.println(" *C");
    delay(1500);                          // 延时 1.5 秒
}
