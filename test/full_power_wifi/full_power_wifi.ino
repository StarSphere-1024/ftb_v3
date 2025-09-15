#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include <PS2X_lib.h>
#include "LIS3DHTR.h"
#include "DHT.h"
#include "Ultrasonic.h"
#include <WonderK210_I2C_Slave.h>

#define RGB_PIN 33
#define NUM_RGB_LEDS 9
#define RGB_BRIGHTNESS 127
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_RGB_LEDS];

#define I2C_SDA 39
#define I2C_SCL 40

#define K210_I2C_SCL_PIN 36
#define K210_I2C_SDA_PIN 37

#define ASR_RX_PIN 35
#define ASR_TX_PIN 34

#define GROVE6_PIN_A 1
#define GROVE6_PIN_B 2
#define GROVE3_PIN_A 3
#define GROVE3_PIN_B 4
#define GROVE5_PIN_A 5
#define GROVE5_PIN_B 6
#define GROVE2_PIN_A 7
#define GROVE2_PIN_B 8
#define GROVE4_PIN_A 26
#define GROVE4_PIN_B 38

#define SENSOR_L1 GROVE3_PIN_A  // 最左
#define SENSOR_L0 GROVE3_PIN_B  // 中左
#define SENSOR_R0 GROVE6_PIN_A  // 中右
#define SENSOR_R1 GROVE6_PIN_B  // 最右

#define LIGHT_PIN GROVE2_PIN_A
#define DHT_PIN GROVE4_PIN_A
#define ULTRASONIC_PIN GROVE5_PIN_A

#define PS2_CMD_PIN 9
#define PS2_DATA_PIN 10
#define PS2_CLK_PIN 41
#define PS2_CS_PIN 42

#define LF_MOTOR_FWD_PWM 12
#define LF_MOTOR_REV_PWM 11
#define RF_MOTOR_FWD_PWM 13
#define RF_MOTOR_REV_PWM 14
#define LR_MOTOR_FWD_PWM 16
#define LR_MOTOR_REV_PWM 15
#define RR_MOTOR_FWD_PWM 17
#define RR_MOTOR_REV_PWM 18

#define LINE_THRESHOLD 500
#define BASE_SPEED 150
#define TURN_OMEGA 50
#define SPIN_OMEGA 120

#define WIFI_AP_SSID "ESP32_S3_AP"

WebServer server(80);

LIS3DHTR<TwoWire> LIS;
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);
Ultrasonic ultrasonic(ULTRASONIC_PIN);
TwoWire *k210Wire = new TwoWire(1);
WonderK210_I2C *k210 = new WonderK210_I2C(k210Wire);
Find_Box_st *result;
PS2X ps2x;

struct SensorData_t
{
    float temperature;
    float humidity;
    long ultrasonic_distance;
    float accel_x, accel_y, accel_z;
    int light_value;
    int line_follower_L1;
    int line_follower_L0;
    int line_follower_R0;
    int line_follower_R1;
    Find_Box_st face_result;
    bool face_detected;
};

SensorData_t g_sensorData;
SemaphoreHandle_t g_sensorDataMutex;
bool g_isAccelerometerPresent = false;
bool g_isK210Connected = false;

void controlWheel(uint8_t fwdPin, uint8_t revPin, int speed)
{
    if (speed > 0)
    {
        analogWrite(fwdPin, speed);
        analogWrite(revPin, 0);
    }
    else if (speed < 0)
    {
        analogWrite(fwdPin, 0);
        analogWrite(revPin, -speed);
    }
    else
    {
        analogWrite(fwdPin, 0);
        analogWrite(revPin, 0);
    }
}

void setMecanumWheels(int lfSpeed, int rfSpeed, int lrSpeed, int rrSpeed)
{
    controlWheel(LF_MOTOR_FWD_PWM, LF_MOTOR_REV_PWM, lfSpeed);
    controlWheel(RF_MOTOR_FWD_PWM, RF_MOTOR_REV_PWM, rfSpeed);
    controlWheel(LR_MOTOR_FWD_PWM, LR_MOTOR_REV_PWM, lrSpeed);
    controlWheel(RR_MOTOR_FWD_PWM, RR_MOTOR_REV_PWM, rrSpeed);
}

/**
 * @brief 控制麦克纳母轮小车的运动
 * @param vx 前进速度 (-255 到 255), 正为前进, 负为后退
 * @param vy 平移速度 (-255 到 255), 正为向右, 负为向左
 * @param omega 旋转速度 (-255 到 255), 正为顺时针, 负为逆时针
 */
void mecanumDrive(int vx, int vy, int omega) {
  int pwm_lf = vx - vy + omega;
  int pwm_rf = vx + vy - omega;
  int pwm_lr = vx + vy + omega;
  int pwm_rr = vx - vy - omega;

  // 查找最大PWM值，用于归一化，防止速度超限
  int max_pwm = 0;
  max_pwm = max(max_pwm, abs(pwm_lf));
  max_pwm = max(max_pwm, abs(pwm_rf));
  max_pwm = max(max_pwm, abs(pwm_lr));
  max_pwm = max(max_pwm, abs(pwm_rr));

  // 如果计算出的最大值超过255，则按比例缩小所有值
  if (max_pwm > 255) {
    pwm_lf = map(pwm_lf, -max_pwm, max_pwm, -255, 255);
    pwm_rf = map(pwm_rf, -max_pwm, max_pwm, -255, 255);
    pwm_lr = map(pwm_lr, -max_pwm, max_pwm, -255, 255);
    pwm_rr = map(pwm_rr, -max_pwm, max_pwm, -255, 255);
  }

  setMecanumWheels(pwm_lf, pwm_rf, pwm_lr, pwm_rr);
}


/**
 * @brief 根据传感器状态执行巡线逻辑
 */
void executeLineFollowingLogic() {
  // 直接读取传感器状态以保证实时性
  bool sL1 = (analogRead(SENSOR_L1) > LINE_THRESHOLD);
  bool sL0 = (analogRead(SENSOR_L0) > LINE_THRESHOLD);
  bool sR0 = (analogRead(SENSOR_R0) > LINE_THRESHOLD);
  bool sR1 = (analogRead(SENSOR_R1) > LINE_THRESHOLD);

  // 将4个传感器的状态合并成一个4位的二进制数
  // 格式: sL1 sL0 sR0 sR1
  byte sensorState = (sL1 << 3) | (sL0 << 2) | (sR0 << 1) | sR1;

  switch (sensorState) {
    case 0b0110: // 状态 [0,1,1,0]: 完美在循迹线上 -> 直行
      mecanumDrive(BASE_SPEED, 0, 0);
      break;

    case 0b0010: // 状态 [0,0,1,0]: 车体偏左 -> 向右轻转
      mecanumDrive(BASE_SPEED, 0, TURN_OMEGA);
      break;
    case 0b0100: // 状态 [0,1,0,0]: 车体偏右 -> 向左轻转
      mecanumDrive(BASE_SPEED, 0, -TURN_OMEGA);
      break;

    case 0b0011: // 状态 [0,0,1,1]: 车体严重偏左 -> 向右转
      mecanumDrive(BASE_SPEED, 0, TURN_OMEGA * 1.2);
    case 0b1100: // 状态 [1,1,0,0]: 车体严重偏右 -> 向左转
      mecanumDrive(BASE_SPEED, 0, -TURN_OMEGA * 1.2);
      break;

    case 0b0001: // 状态 [0,0,0,1]: 车体在最左侧，压到最右侧线 -> 向右急转
     mecanumDrive(BASE_SPEED, 0, TURN_OMEGA * 1.5);
    case 0b1000: // 状态 [1,0,0,0]: 车体在最右侧，压到最左侧线 -> 向左急转
     mecanumDrive(BASE_SPEED, 0, -TURN_OMEGA * 1.5);
      break;

    // 处理直角弯或T字路口
    case 0b1110: // 左侧检测到横线 -> 向左旋转
       mecanumDrive(0, 0, -SPIN_OMEGA);
       break;

    case 0b0111: // 右侧检测到横线 -> 向右旋转
       mecanumDrive(0, 0, SPIN_OMEGA);
       break;

    case 0b0000: 
      mecanumDrive(0, 0, SPIN_OMEGA);
      break;

    case 0b1111: 
      mecanumDrive(0, 0, 0);
      break;

    default: // 其他未定义状态
      // 默认停止，保证安全
      mecanumDrive(0, 0, 0);
      break;
  }
}

void sendToASR(String command)
{
    Serial2.println(command);
}

void vLineFollowingTask(void *pvParameters)
{
    (void)pvParameters;
    Serial.println("Line Following Task started.");
    for (;;)
    {
        executeLineFollowingLogic();
        vTaskDelay(pdMS_TO_TICKS(20)); // 循线逻辑可以运行得更频繁一些
    }
}



void vVoiceTask(void *pvParameters)
{
    (void)pvParameters;
    Serial.println("Voice Task started.");
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        Serial.println("Voice Task: Playing sound.");
        sendToASR("PlayVol:100\n");
    }
}

void vRgbTask(void *pvParameters)
{
    (void)pvParameters;
    Serial.println("RGB Task started: Setting constant white");
    fill_solid(leds, NUM_RGB_LEDS, CRGB::White);
    FastLED.show();
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vSensorUpdateTask(void *pvParameters)
{
    (void)pvParameters;
    Serial.println("Sensor Update Task started.");
    float temp_hum_val[2] = {0};
    long distance;
    float ax, ay, az;
    int light;
    int L1, L0, R0, R1;
    Find_Box_st face_res;
    bool face_found;

    for (;;)
    {
        temp_hum_val[0] = dht.readHumidity();
        temp_hum_val[1] = dht.readTemperature();
        if (isnan(temp_hum_val[0]) || isnan(temp_hum_val[1]))
        {
            temp_hum_val[0] = -1;
            temp_hum_val[1] = -1;
        }
        distance = ultrasonic.MeasureInCentimeters();
        if (g_isAccelerometerPresent)
        {
            ax = LIS.getAccelerationX();
            ay = LIS.getAccelerationY();
            az = LIS.getAccelerationZ();
        }
        light = analogRead(LIGHT_PIN);
        L1 = analogRead(SENSOR_L1);
        L0 = analogRead(SENSOR_L0);
        R0 = analogRead(SENSOR_R0);
        R1 = analogRead(SENSOR_R1);
        if (g_isK210Connected)
        {
            k210->update_data();
            face_found = k210->recive_box(result, K210_FIND_FACE_YOLO);
            if (face_found)
            {
                face_res = *result;
            }
        }
        else
        {
            face_found = false;
        }

        if (xSemaphoreTake(g_sensorDataMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            g_sensorData.humidity = temp_hum_val[0];
            g_sensorData.temperature = temp_hum_val[1];
            g_sensorData.ultrasonic_distance = distance;
            g_sensorData.accel_x = ax;
            g_sensorData.accel_y = ay;
            g_sensorData.accel_z = az;
            g_sensorData.light_value = light;
            g_sensorData.line_follower_L1 = L1;
            g_sensorData.line_follower_L0 = L0;
            g_sensorData.line_follower_R0 = R0;
            g_sensorData.line_follower_R1 = R1;
            g_sensorData.face_detected = face_found;
            if (face_found)
            {
                g_sensorData.face_result = face_res;
            }
            xSemaphoreGive(g_sensorDataMutex);
        }
        else
        {
            Serial.println("Sensor task failed to get mutex!");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void vSerialReportTask(void *pvParameters)
{
    (void)pvParameters;
    Serial.println("Serial Report Task started.");
    SensorData_t localSensorData;

    for (;;)
    {
        if (xSemaphoreTake(g_sensorDataMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            localSensorData = g_sensorData;
            xSemaphoreGive(g_sensorDataMutex);
        }
        else
        {
            Serial.println("Serial task failed to get mutex!");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        printf("\n--- Full-Load Test Sensor Report ---\n");
        printf("Timestamp: %lu\n", millis());
        printf("Temp: %.2f *C\n", localSensorData.temperature);
        printf("Humidity: %.2f %%\n", localSensorData.humidity);
        printf("Ultrasonic: %ld cm\n", localSensorData.ultrasonic_distance);
        printf("Light: %d\n", localSensorData.light_value);
        printf("Accelerometer: x=%.2f, y=%.2f, z=%.2f\n", localSensorData.accel_x, localSensorData.accel_y, localSensorData.accel_z);

        printf("Line Follower: L1=%d, L0=%d, R0=%d, R1=%d\n",
               localSensorData.line_follower_L1,
               localSensorData.line_follower_L0,
               localSensorData.line_follower_R0,
               localSensorData.line_follower_R1);

        if (localSensorData.face_detected)
        {
            printf("Face Detected: Yes | x=%d, y=%d, w=%d, h=%d\n",
                   localSensorData.face_result.x,
                   localSensorData.face_result.y,
                   localSensorData.face_result.w,
                   localSensorData.face_result.h);
        }
        else
        {
            Serial.println("Face Detected: No");
        }
        Serial.println("-------------------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void handleSensorData()
{
    SensorData_t localSensorData;
    if (xSemaphoreTake(g_sensorDataMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        localSensorData = g_sensorData;
        xSemaphoreGive(g_sensorDataMutex);
    }
    else
    {
        server.send(503, "application/json", "{\"error\":\"Failed to get sensor data\"}");
        return;
    }

    StaticJsonDocument<512> doc;
    doc["timestamp"] = millis();
    doc["temperature"] = localSensorData.temperature;
    doc["humidity"] = localSensorData.humidity;
    doc["ultrasonic_distance"] = localSensorData.ultrasonic_distance;
    doc["light_value"] = localSensorData.light_value;
    JsonObject accel = doc.createNestedObject("accelerometer");
    accel["x"] = localSensorData.accel_x;
    accel["y"] = localSensorData.accel_y;
    accel["z"] = localSensorData.accel_z;
    JsonObject line_follower = doc.createNestedObject("line_follower");
    line_follower["L1"] = localSensorData.line_follower_L1;
    line_follower["L0"] = localSensorData.line_follower_L0;
    line_follower["R0"] = localSensorData.line_follower_R0;
    line_follower["R1"] = localSensorData.line_follower_R1;
    doc["face_detected"] = localSensorData.face_detected;
    if (localSensorData.face_detected)
    {
        JsonObject face_result = doc.createNestedObject("face_result");
        face_result["x"] = localSensorData.face_result.x;
        face_result["y"] = localSensorData.face_result.y;
        face_result["w"] = localSensorData.face_result.w;
        face_result["h"] = localSensorData.face_result.h;
    }

    String jsonString;
    serializeJson(doc, jsonString);
    server.send(200, "application/json", jsonString);
}

void vWifiServerTask(void *pvParameters)
{
    (void)pvParameters;
    Serial.println("WiFi Server Task started.");
    for (;;)
    {
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    delay(1000);
    Serial.println("\n\nESP32-S3 Full-Load Test Program Starting...");
    Serial2.begin(115200, SERIAL_8N1, ASR_RX_PIN, ASR_TX_PIN);
    sendToASR("SetVolume:7\n");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.beginTransmission(0x19);
    if (Wire.endTransmission() == 0)
    {
        g_isAccelerometerPresent = true;
        Serial.println("LIS3DHTR Accelerometer found. Initializing...");
        LIS.begin(Wire, 0x19);
        LIS.setFullScaleRange(LIS3DHTR_RANGE_2G);
        LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
        Serial.println("LIS3DHTR configured.");
    }
    else
    {
        g_isAccelerometerPresent = false;
        Serial.println("WARNING: LIS3DHTR Accelerometer not found.");
    }
    ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_CS_PIN, PS2_DATA_PIN, true, true);
    k210->begin(K210_I2C_SDA_PIN, K210_I2C_SCL_PIN);
    if (k210->isConnected())
    {
        g_isK210Connected = true;
        Serial.println("K210 found and initialized.");
    }
    else
    {
        g_isK210Connected = false;
        Serial.println("WARNING: K210 not found.");
    }
    result = new Find_Box_st();
    pinMode(LIGHT_PIN, INPUT);
    pinMode(GROVE3_PIN_A, INPUT);
    pinMode(GROVE3_PIN_B, INPUT);
    pinMode(GROVE6_PIN_A, INPUT);
    pinMode(GROVE6_PIN_B, INPUT);

    uint8_t motorPins[] = {LF_MOTOR_FWD_PWM, LF_MOTOR_REV_PWM, RF_MOTOR_FWD_PWM, RF_MOTOR_REV_PWM,
                           LR_MOTOR_FWD_PWM, LR_MOTOR_REV_PWM, RR_MOTOR_FWD_PWM, RR_MOTOR_REV_PWM};
    for (uint8_t pin : motorPins)
    {
        pinMode(pin, OUTPUT);
        analogWriteFrequency(pin, 5000);
    }

    FastLED.addLeds<LED_TYPE, RGB_PIN, COLOR_ORDER>(leds, NUM_RGB_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(RGB_BRIGHTNESS);
    fill_solid(leds, NUM_RGB_LEDS, CRGB::Black);
    FastLED.show();

    Serial.println("\nInitializing WiFi and starting Access Point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID);
    Serial.print("AP SSID: ");
    Serial.println(WIFI_AP_SSID);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    server.on("/sensors", handleSensorData);
    server.begin();
    Serial.println("HTTP server started. Access /sensors for data.");

    Serial.println("Hardware initialized.");

    g_sensorDataMutex = xSemaphoreCreateMutex();
    if (g_sensorDataMutex == NULL)
    {
        Serial.println("Failed to create mutex!");
        while (1)
            ;
    }

    Serial.println("Creating FreeRTOS tasks...");

    xTaskCreate(vLineFollowingTask, "LineFollowingTask", 2048, NULL, 2, NULL);
    xTaskCreate(vVoiceTask, "VoiceTask", 2048, NULL, 1, NULL);
    xTaskCreate(vRgbTask, "RgbTask", 2048, NULL, 1, NULL);
    xTaskCreate(vSensorUpdateTask, "SensorUpdateTask", 4096, NULL, 3, NULL);
    xTaskCreate(vSerialReportTask, "SerialReportTask", 4096, NULL, 1, NULL);
    xTaskCreate(vWifiServerTask, "WifiServerTask", 4096, NULL, 1, NULL);

    Serial.println("All tasks created. Full-load test is running.");
}

void loop()
{
    vTaskDelete(NULL);
}