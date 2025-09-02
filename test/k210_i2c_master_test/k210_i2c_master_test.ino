#include <Wire.h>
#include "WonderK210_I2C_Master.h" // 确保 WonderK210_I2C_Master.h 和 .cpp 文件与此 ino 文件在同一目录下

// ================= ESP32-S3 I2C Slave 配置 =================
// I2C 从机地址，必须与 K210 Python 代码中的 SLAVE_ADDR 一致
#define I2C_SLAVE_ADDR 0x24

// ESP32-S3 I2C 引脚配置 (请根据实际接线修改)
const int sdaPin = 36;
const int sclPin = 37;
// ==========================================================

// 实例化 WonderK210_I2C 对象
WonderK210_I2C k210(I2C_SLAVE_ADDR);

// 创建一个结构体来存储接收到的数据
Find_Box_st face_box;

void setup() {
  // 启动串口用于调试信息输出
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\nESP32-S3 I2C Slave Example");

  // 初始化 I2C 总线并设置引脚
  // 对于 ESP32，可以在 Wire.begin 之前调用 setPins
  Wire.setPins(sdaPin, sclPin);
  
  // 初始化 WonderK210 I2C 库
  // 这会启动 I2C 从机并注册接收回调函数
  k210.begin();
}

void loop() {
  // 在主循环中不断调用 update_data()
  // 它会处理 I2C 中断接收到的数据，并进行协议解析
  

  // 尝试获取人脸识别数据
  // recive_box 函数会检查是否有匹配功能号 (K210_FIND_FACE_YOLO) 的完整数据包
  if (k210.recive_box(&face_box, K210_FIND_FACE_YOLO)) {
    Serial.println("-------------------------");
    Serial.println("Received Face Box Data!");
    Serial.printf("X: %d\n", face_box.x);
    Serial.printf("Y: %d\n", face_box.y);
    Serial.printf("W: %d\n", face_box.w);
    Serial.printf("H: %d\n", face_box.h);
    Serial.println("-------------------------");
  }
  
  delay(10); // 短暂延时，避免 loop 占用全部 CPU
}