#include <Arduino.h>

// 舵机相关引脚和定时参数定义
#define SERVO1_PIN 48
#define SERVO2_PIN 47
#define SERVO_TIMER_FREQUENCY 1000000
#define SERVO_TIMER_TICK_US 50
#define SERVO_PWM_PERIOD_MS 20
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500

#define SERVO_PWM_PERIOD_TICKS (SERVO_PWM_PERIOD_MS * 1000 / SERVO_TIMER_TICK_US)
volatile uint16_t g_servo1_pulse_ticks, g_servo2_pulse_ticks;  // 当前舵机脉冲宽度（tick）


// 定时器和互斥锁
hw_timer_t *g_servo_timer = NULL;
portMUX_TYPE g_servo_timer_mux = portMUX_INITIALIZER_UNLOCKED;


void ARDUINO_ISR_ATTR on_servo_timer() {
  portENTER_CRITICAL_ISR(&g_servo_timer_mux);
  static uint16_t counter = 0;
  if (counter == 0) {
    digitalWrite(SERVO1_PIN, HIGH);
    digitalWrite(SERVO2_PIN, HIGH);
  }
  if (counter >= g_servo1_pulse_ticks) {
    digitalWrite(SERVO1_PIN, LOW);
  }
  if (counter >= g_servo2_pulse_ticks) {
    digitalWrite(SERVO2_PIN, LOW);
  }
  counter++;
  if (counter >= SERVO_PWM_PERIOD_TICKS) {
    counter = 0;
  }
  portEXIT_CRITICAL_ISR(&g_servo_timer_mux);
}


// 舵机初始化，设置引脚和定时器
void initServo() {
  pinMode(SERVO1_PIN, OUTPUT);
  pinMode(SERVO2_PIN, OUTPUT);
  g_servo_timer = timerBegin(SERVO_TIMER_FREQUENCY);
  timerAttachInterrupt(g_servo_timer, &on_servo_timer);
  timerAlarm(g_servo_timer, SERVO_TIMER_TICK_US, true, 0);
}


// 设置指定舵机角度
void Servos_SetAngle(uint8_t servo_num, uint8_t angle) {
  angle = constrain(angle, 0, 180);                                                // 限制角度范围
  uint32_t pulse_us = map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);  // 角度转脉冲宽度
  uint16_t pulse_ticks = pulse_us / SERVO_TIMER_TICK_US;                           // 脉冲宽度转tick
  portENTER_CRITICAL(&g_servo_timer_mux);
  if (servo_num == 1)
    g_servo1_pulse_ticks = pulse_ticks;
  else if (servo_num == 2)
    g_servo2_pulse_ticks = pulse_ticks;
  portEXIT_CRITICAL(&g_servo_timer_mux);
}


// 初始化串口和舵机，设置不同角度
void setup() {
  Serial.begin(9600);
  initServo();  // 初始化舵机
  Servos_SetAngle(1, 75);
  Servos_SetAngle(2, 90);
  delay(1000);
}

void loop() {
  // 依次设置舵机角度
  Servos_SetAngle(1, 0);
  delay(1000);
  Servos_SetAngle(1, 75);
  delay(1000);
  Servos_SetAngle(2, 0);
  delay(1000);
  Servos_SetAngle(2, 180);
  delay(1000);
  Servos_SetAngle(2, 90);
  delay(1000);
}
