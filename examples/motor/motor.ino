#include <Arduino.h>

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

// 用结构体描述每个电机的PWM引脚
struct Motor
{
  uint8_t fwdPin;
  uint8_t revPin;
};

// 四个麦克纳姆轮电机（左前、右前、左后、右后）
Motor motors[4] = {
    {12, 11}, // 左前轮
    {13, 14}, // 右前轮
    {16, 15}, // 左后轮
    {17, 18}  // 右后轮
};

enum CarState
{
  STOP,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  CLOCKWISE,
  COUNTER_CLOCKWISE,
  BRAKING
} CurrentState = STOP;

const uint8_t MIN_SPEED = 100;

/**
 * @brief 控制单个轮子的运动。
 *
 * @param fwdPin 轮子前进方向的PWM引脚。
 * @param revPin 轮子后退方向的PWM引脚。
 * @param speed 速度值 (0-255)。正值表示前进，负值表示后退，0表示停止。
 */

void controlWheel(const Motor &motor, int speed)
{
  if (speed > 0)
  {
    analogWrite(motor.fwdPin, speed);
    analogWrite(motor.revPin, 0);
  }
  else if (speed < 0)
  {
    analogWrite(motor.fwdPin, 0);
    analogWrite(motor.revPin, -speed);
  }
  else
  {
    analogWrite(motor.fwdPin, 0);
    analogWrite(motor.revPin, 0);
  }
}

/**
 * @brief 设置所有电机PWM引脚的频率。
 * @param frequency PWM频率 (Hz)。
 */

void setAllPwmFrequencies(int frequency)
{
  for (int i = 0; i < 4; ++i)
  {
    analogWriteFrequency(motors[i].fwdPin, frequency);
    analogWriteFrequency(motors[i].revPin, frequency);
  }
}

/**
 * @brief 控制所有轮子运动，用于实现麦克纳姆轮的复合运动。
 *
 * @param lfSpeed 左前轮速度 (-255到255)。
 * @param rfSpeed 右前轮速度 (-255到255)。
 * @param lrSpeed 左后轮速度 (-255到255)。
 * @param rrSpeed 右后轮速度 (-255到255)。
 */

void setMecanumWheels(int lfSpeed, int rfSpeed, int lrSpeed, int rrSpeed)
{
  int speeds[4] = {lfSpeed, rfSpeed, lrSpeed, rrSpeed};
  for (int i = 0; i < 4; ++i)
  {
    controlWheel(motors[i], speeds[i]);
  }
}

/**
 * @brief 让机器人向前移动。
 * @param speed 速度 (0-255)。
 */
void moveForward(uint8_t speed)
{
  CurrentState = FORWARD;
  // 麦克纳姆轮前进：所有轮子同向转动
  setMecanumWheels(speed, speed, speed, speed);
}

/**
 * @brief 让机器人向后移动。
 * @param speed 速度 (0-255)。
 */
void moveBackward(uint8_t speed)
{
  CurrentState = BACKWARD;
  // 麦克纳姆轮后退：所有轮子反向转动
  setMecanumWheels(-speed, -speed, -speed, -speed);
}

/**
 * @brief 让机器人向左平移。
 * @param speed 速度 (0-255)。
 */
void strafeLeft(uint8_t speed)
{
  CurrentState = LEFT;
  // 麦克纳姆轮左平移：左前、右后轮后退；右前、左后轮前进
  setMecanumWheels(-speed, speed, speed, -speed);
}

/**
 * @brief 让机器人向右平移。
 * @param speed 速度 (0-255)。
 */
void strafeRight(uint8_t speed)
{
  CurrentState = RIGHT;
  // 麦克纳姆轮右平移：左前、右后轮前进；右前、左后轮后退
  setMecanumWheels(speed, -speed, -speed, speed);
}

/**
 * @brief 让机器人顺时针旋转。
 * @param speed 速度 (0-255)。
 */
void rotateClockwise(uint8_t speed)
{
  CurrentState = CLOCKWISE;
  // 顺时针旋转：左侧轮前进，右侧轮后退
  setMecanumWheels(speed, -speed, speed, -speed);
}

/**
 * @brief 让机器人逆时针旋转。
 * @param speed 速度 (0-255)。
 */
void rotateCounterClockwise(uint8_t speed)
{
  CurrentState = COUNTER_CLOCKWISE;
  // 逆时针旋转：左侧轮后退，右侧轮前进
  setMecanumWheels(-speed, speed, -speed, speed);
}

/**
 * @brief 释放所有轮子的动力，使其自由滚动。
 */
void release()
{
  CurrentState = STOP;
  setMecanumWheels(0, 0, 0, 0);
}

/**
 * @brief 刹车，通过反向瞬时施加制动力。
 * @param brakeDuration 刹车持续时间 (毫秒)。
 */
void brake(unsigned int brakeDuration = 100)
{
  if (CurrentState == FORWARD)
  {
    moveBackward(255);
  }
  else if (CurrentState == BACKWARD)
  {
    moveForward(255);
  }
  else if (CurrentState == LEFT)
  {
    strafeRight(255);
  }
  else if (CurrentState == RIGHT)
  {
    strafeLeft(255);
  }
  else if (CurrentState == CLOCKWISE)
  {
    rotateCounterClockwise(255);
  }
  else if (CurrentState == COUNTER_CLOCKWISE)
  {
    rotateClockwise(255);
  }
  else
  {
    // 如果当前是停止状态或者其他未知状态，直接释放
    release();
    return;
  }
  delay(brakeDuration);
  release();
  CurrentState = BRAKING; // 设置为刹车状态，结束后再设为STOP
}

void setup()
{
  Serial.begin(9600);
  // 初始化所有PWM引脚为输出
  for (int i = 0; i < 4; ++i)
  {
    pinMode(motors[i].fwdPin, OUTPUT);
    pinMode(motors[i].revPin, OUTPUT);
  }
  setAllPwmFrequencies(5000);
}

void loop()
{

  Serial.println("Moving Forward...");
  // 以中等速度前进
  moveForward(200);delay(2000);
  release();delay(500);

  Serial.println("Moving Backward...");
  // 以中等速度后退
  moveBackward(200); delay(2000);
  release(); delay(500);
}
