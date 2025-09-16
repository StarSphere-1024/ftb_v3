// 基础速度
#define BASE_SPEED 150          // 巡线时的基础前进速度
// 转向控制 (omega值)
#define TURN_OMEGA 50           // 轻微转向时的旋转速度分量
#define SPIN_OMEGA 120           // 原地旋转时的旋转速度分量

// 传感器阈值
#define LINE_THRESHOLD 500

// 传感器引脚定义
#define SENSOR_L1 3  // 最左
#define SENSOR_L0 4  // 中左
#define SENSOR_R0 1  // 中右
#define SENSOR_R1 2  // 最右

#define LF_MOTOR_FWD_PWM 12 // 左前-前进
#define LF_MOTOR_REV_PWM 11 // 左前-后退
#define RF_MOTOR_FWD_PWM 13 // 右前-前进
#define RF_MOTOR_REV_PWM 14 // 右前-后退
#define LR_MOTOR_FWD_PWM 16 // 左后-前进
#define LR_MOTOR_REV_PWM 15 // 左后-后退
#define RR_MOTOR_FWD_PWM 17 // 右后-前进
#define RR_MOTOR_REV_PWM 18 // 右后-后退

#define PWM_CHANNELS 4
#define PWM_STEP 30
#define PWM_INTERVAL 10  // ms

unsigned long lastUpdate = 0;
int current_pwm[PWM_CHANNELS] = { 0 };
int target_pwm[PWM_CHANNELS] = { 0 };

int motor_pwm_pins[PWM_CHANNELS][2] = {
  { LF_MOTOR_FWD_PWM, LF_MOTOR_REV_PWM },  // 0: 左前轮 (LF)
  { RF_MOTOR_FWD_PWM, RF_MOTOR_REV_PWM },  // 1: 右前轮 (RF)
  { LR_MOTOR_FWD_PWM, LR_MOTOR_REV_PWM },  // 2: 左后轮 (LR)
  { RR_MOTOR_FWD_PWM, RR_MOTOR_REV_PWM },  // 3: 右后轮 (RR)
};


// 设置单个电机的目标PWM值
void setMotor(int channel, int pwm_value) {
  target_pwm[channel] = constrain(pwm_value, -255, 255);
}

// 实际应用PWM值到电机驱动引脚
void applyMotorPWM(int channel, int value) {
  int pinA = motor_pwm_pins[channel][0];
  int pinB = motor_pwm_pins[channel][1];
  if (value > 0) {
    analogWrite(pinA, value);
    analogWrite(pinB, 0);
  } else if (value < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -value);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}

// 平滑更新所有电机的PWM输出
void updateMotors() {
  if (millis() - lastUpdate < PWM_INTERVAL) return;
  lastUpdate = millis();

  for (int i = 0; i < PWM_CHANNELS; i++) {
    if (target_pwm[i] == current_pwm[i]) continue;
    int diff = target_pwm[i] - current_pwm[i];
    current_pwm[i] += (abs(diff) <= PWM_STEP) ? diff : (diff > 0 ? PWM_STEP : -PWM_STEP);
    applyMotorPWM(i, current_pwm[i]);
  }
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

  // 将计算好的PWM值设置给对应的电机
  setMotor(0, pwm_lf); // 左前
  setMotor(1, pwm_rf); // 右前
  setMotor(2, pwm_lr); // 左后
  setMotor(3, pwm_rr); // 右后
}


/**
 * @brief 根据传感器状态执行巡线逻辑
 */
void executeLineFollowingLogic() {
  // 读取所有传感器状态 (1表示在黑线上, 0表示在白底上)
  bool sL1 = (analogRead(SENSOR_L1) > LINE_THRESHOLD);
  bool sL0 = (analogRead(SENSOR_L0) > LINE_THRESHOLD);
  bool sR0 = (analogRead(SENSOR_R0) > LINE_THRESHOLD);
  bool sR1 = (analogRead(SENSOR_R1) > LINE_THRESHOLD);

  // 将4个传感器的状态合并成一个4位的二进制数
  // 格式: sL1 sL0 sR0 sR1
  byte sensorState = (sL1 << 3) | (sL0 << 2) | (sR0 << 1) | sR1;

  Serial.print("Sensor State (L1,L0,R0,R1): ");
  Serial.print(sL1); Serial.print(sL0); Serial.print(sR0); Serial.println(sR1);

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


void setup() {
  Serial.begin(115200);

  pinMode(SENSOR_L0, INPUT);
  pinMode(SENSOR_R0, INPUT);
  pinMode(SENSOR_L1, INPUT);
  pinMode(SENSOR_R1, INPUT);

  for (int i = 0; i < PWM_CHANNELS; i++) {
    pinMode(motor_pwm_pins[i][0], OUTPUT);
    pinMode(motor_pwm_pins[i][1], OUTPUT);
  }
  Serial.println("Mecanum Line Follower Initialized.");
}

void loop() {
  executeLineFollowingLogic();
  updateMotors();
}