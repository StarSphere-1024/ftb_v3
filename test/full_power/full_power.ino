#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <Wire.h>
#include <WiFi.h>
#include <FastLED.h>
#include <PS2X_lib.h>
#include "LIS3DHTR.h"
#include "DHT.h"
#include "Ultrasonic.h"
#include <WonderK210_I2C_Slave.h>

#define RGB_PIN 33
#define NUM_RGB_LEDS 9
#define RGB_BRIGHTNESS 255
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

#define SERVO1_PIN 48
#define SERVO2_PIN 47
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#define SERVO1_MIN_ANGLE 0
#define SERVO1_MAX_ANGLE 150
#define SERVO2_MIN_ANGLE 0
#define SERVO2_MAX_ANGLE 180
#define SERVO1_RESET_ANGLE (SERVO1_MAX_ANGLE+SERVO1_MIN_ANGLE)/2
#define SERVO2_RESET_ANGLE (SERVO1_MAX_ANGLE+SERVO1_MIN_ANGLE)/2
#define SERVO1_MID_ANGLE (SERVO1_MAX_ANGLE+SERVO1_MIN_ANGLE)/2
#define SERVO2_MID_ANGLE (SERVO1_MAX_ANGLE+SERVO1_MIN_ANGLE)/2

#define WIFI_AP_SSID "ESP32_S3_AP"

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
    Find_Box_st face_result;
    bool face_detected;
};

SensorData_t g_sensorData;
SemaphoreHandle_t g_sensorDataMutex;
bool g_isAccelerometerPresent = false;

hw_timer_t *g_servo_timer = NULL;
portMUX_TYPE g_servo_timer_mux = portMUX_INITIALIZER_UNLOCKED;
#define SERVO_TIMER_FREQUENCY 1000000
#define SERVO_TIMER_TICK_US 50
#define SERVO_PWM_PERIOD_MS 20
#define SERVO_PWM_PERIOD_TICKS (SERVO_PWM_PERIOD_MS * 1000 / SERVO_TIMER_TICK_US)
volatile uint16_t g_servo1_pulse_ticks, g_servo2_pulse_ticks;

void ARDUINO_ISR_ATTR on_servo_timer()
{
    portENTER_CRITICAL_ISR(&g_servo_timer_mux);
    static uint16_t counter = 0;
    if (counter == 0)
    {
        digitalWrite(SERVO1_PIN, HIGH);
        digitalWrite(SERVO2_PIN, HIGH);
    }
    if (counter == g_servo1_pulse_ticks)
    {
        digitalWrite(SERVO1_PIN, LOW);
    }
    if (counter == g_servo2_pulse_ticks)
    {
        digitalWrite(SERVO2_PIN, LOW);
    }
    counter++;
    if (counter >= SERVO_PWM_PERIOD_TICKS)
    {
        counter = 0;
    }
    portEXIT_CRITICAL_ISR(&g_servo_timer_mux);
}

void Servos_SetAngle(uint8_t servo_num, uint8_t angle)
{
    angle = constrain(angle, 0, 180);
    uint32_t pulse_us = map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    uint16_t pulse_ticks = pulse_us / SERVO_TIMER_TICK_US;
    portENTER_CRITICAL(&g_servo_timer_mux);
    if (servo_num == 1)
        g_servo1_pulse_ticks = pulse_ticks;
    else if (servo_num == 2)
        g_servo2_pulse_ticks = pulse_ticks;
    portEXIT_CRITICAL(&g_servo_timer_mux);
}

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

void moveForward(uint8_t speed)
{
    setMecanumWheels(speed, speed, speed, speed);
}

void sendToASR(String command)
{
    Serial2.println(command);
}

void vMotorTask(void *pvParameters)
{
    (void)pvParameters;
    Serial.println("Motor Task started: Ramping up speed over 5 seconds...");

    const int MAX_SPEED = 255;
    const int RAMP_UP_TIME_MS = 3000;
    const int delay_per_step = RAMP_UP_TIME_MS / MAX_SPEED;

    for (int speed = 0; speed <= MAX_SPEED; speed++)
    {
        moveForward(speed);
        vTaskDelay(pdMS_TO_TICKS(delay_per_step));
    }

    Serial.println("Motor Task: Reached max speed. Maintaining.");
    moveForward(MAX_SPEED);

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void vServoTask(void *pvParameters)
{
    (void)pvParameters;
    Serial.println("Servo Task started.");
    uint8_t servo1_angle[] = {SERVO1_MIN_ANGLE, SERVO1_MID_ANGLE, SERVO1_MAX_ANGLE, SERVO1_MID_ANGLE};
    uint8_t servo2_angle[] = {SERVO2_MIN_ANGLE, SERVO2_MID_ANGLE, SERVO2_MAX_ANGLE, SERVO2_MID_ANGLE};
    for (;;)
    {
        for (int i = 0; i < 4; i++)
        {
            Servos_SetAngle(1, servo1_angle[i]);
            Servos_SetAngle(2, servo2_angle[i]);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
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
        k210->update_data();
        face_found = k210->recive_box(result, K210_FIND_FACE_YOLO);
        if (face_found)
        {
            face_res = *result;
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

        Serial.println("\n--- Full-Load Test Sensor Report ---");
        Serial.print("Timestamp: ");
        Serial.println(millis());
        Serial.print("Temp: ");
        Serial.print(localSensorData.temperature);
        Serial.println(" *C");
        Serial.print("Humidity: ");
        Serial.print(localSensorData.humidity);
        Serial.println(" %");
        Serial.print("Ultrasonic: ");
        Serial.print(localSensorData.ultrasonic_distance);
        Serial.println(" cm");
        Serial.print("Light: ");
        Serial.println(localSensorData.light_value);
        Serial.print("Accelerometer: x=");
        Serial.print(localSensorData.accel_x, 2);
        Serial.print(", y=");
        Serial.print(localSensorData.accel_y, 2);
        Serial.print(", z=");
        Serial.println(localSensorData.accel_z, 2);

        if (localSensorData.face_detected)
        {
            Serial.print("Face Detected: Yes | x=");
            Serial.print(localSensorData.face_result.x);
            Serial.print(", y=");
            Serial.print(localSensorData.face_result.y);
            Serial.print(", w=");
            Serial.print(localSensorData.face_result.w);
            Serial.print(", h=");
            Serial.println(localSensorData.face_result.h);
        }
        else
        {
            Serial.println("Face Detected: No");
        }
        Serial.println("-------------------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    delay(1000);
    Serial.println("\n\nESP32-S3 Full-Load Test Program Starting...");
    Serial2.begin(115200, SERIAL_8N1, ASR_RX_PIN, ASR_TX_PIN);

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
    result = new Find_Box_st();
    pinMode(LIGHT_PIN, INPUT);

    pinMode(SERVO1_PIN, OUTPUT);
    pinMode(SERVO2_PIN, OUTPUT);
    g_servo_timer = timerBegin(SERVO_TIMER_FREQUENCY);
    timerAttachInterrupt(g_servo_timer, &on_servo_timer);
    timerAlarm(g_servo_timer, SERVO_TIMER_TICK_US, true, 0);
    Servos_SetAngle(1, SERVO1_RESET_ANGLE);
    Servos_SetAngle(2, SERVO2_RESET_ANGLE);

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

    Serial.println("Hardware initialized.");

    g_sensorDataMutex = xSemaphoreCreateMutex();
    if (g_sensorDataMutex == NULL)
    {
        Serial.println("Failed to create mutex!");
        while (1)
            ;
    }

    Serial.println("Creating FreeRTOS tasks...");

    xTaskCreate(vMotorTask, "MotorTask", 2048, NULL, 2, NULL);
    xTaskCreate(vServoTask, "ServoTask", 2048, NULL, 2, NULL);
    xTaskCreate(vVoiceTask, "VoiceTask", 2048, NULL, 1, NULL);
    xTaskCreate(vRgbTask, "RgbTask", 2048, NULL, 1, NULL);
    xTaskCreate(vSensorUpdateTask, "SensorUpdateTask", 4096, NULL, 3, NULL);
    xTaskCreate(vSerialReportTask, "SerialReportTask", 4096, NULL, 1, NULL);

    Serial.println("All tasks created. Full-load test is running.");
}

void loop()
{
    vTaskDelete(NULL);
}