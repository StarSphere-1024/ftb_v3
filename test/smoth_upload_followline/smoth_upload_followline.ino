// 8 路电机控制，使用 -255 ~ +255 的有符号 PWM 模型
#define LINE_FOLLOW_BASE_SPEED 130
#define LIGHT_TURN_ADJUST 30  // 轻转调整（小偏差）
#define HARD_TURN_ADJUST 60   // 急转调整（大偏差）
#define LIGHT_TURN_SPEED_FAST (LINE_FOLLOW_BASE_SPEED + LIGHT_TURN_ADJUST)
#define LIGHT_TURN_SPEED_SLOW (LINE_FOLLOW_BASE_SPEED - LIGHT_TURN_ADJUST)
#define HARD_TURN_SPEED_FAST (LINE_FOLLOW_BASE_SPEED + HARD_TURN_ADJUST)
#define HARD_TURN_SPEED_SLOW (LINE_FOLLOW_BASE_SPEED - HARD_TURN_ADJUST)
#define SEARCH_ROTATE_SPEED (LINE_FOLLOW_BASE_SPEED - 20)

#define LEFT1 3
#define LEFT0 4
#define RIGHT0 1
#define RIGHT1 2

#define LF_MOTOR_FWD_PWM 12
#define LF_MOTOR_REV_PWM 11
#define RF_MOTOR_FWD_PWM 13
#define RF_MOTOR_REV_PWM 14
#define LR_MOTOR_FWD_PWM 16
#define LR_MOTOR_REV_PWM 15
#define RR_MOTOR_FWD_PWM 17
#define RR_MOTOR_REV_PWM 18

int towar_left_0 = 0;
int towar_right_0 = 0;
int towar_left_1 = 0;
int towar_right_1 = 0;

#define LINE_THRESHOLD 400

#define PWM_CHANNELS 4
#define PWM_STEP 30
#define PWM_INTERVAL 10  // ms

unsigned long lastUpdate = 0;
int current_pwm[PWM_CHANNELS] = { 0 };
int target_pwm[PWM_CHANNELS] = { 0 };

int motor_pwm_pins[PWM_CHANNELS][2] = {
  { LF_MOTOR_FWD_PWM, LF_MOTOR_REV_PWM },  // 前左
  { RF_MOTOR_FWD_PWM, RF_MOTOR_REV_PWM },  // 前右
  { LR_MOTOR_FWD_PWM, LR_MOTOR_REV_PWM },  // 后左
  { RR_MOTOR_FWD_PWM, RR_MOTOR_REV_PWM },  // 后右
};

void setup() {
  Serial.begin(115200);

  pinMode(LEFT0, INPUT);
  pinMode(RIGHT0, INPUT);
  pinMode(LEFT1, INPUT);
  pinMode(RIGHT1, INPUT);

  for (int i = 0; i < PWM_CHANNELS; i++) {
    pinMode(motor_pwm_pins[i][0], OUTPUT);
    pinMode(motor_pwm_pins[i][1], OUTPUT);
  }
}

void loop() {
  towar_left_1 = (analogRead(LEFT1) > LINE_THRESHOLD) ? 1 : 0;
  towar_left_0 = (analogRead(LEFT0) > LINE_THRESHOLD) ? 1 : 0;
  towar_right_0 = (analogRead(RIGHT0) > LINE_THRESHOLD) ? 1 : 0;
  towar_right_1 = (analogRead(RIGHT1) > LINE_THRESHOLD) ? 1 : 0;
  Serial.print("Sensors: L1=");
  Serial.print(towar_left_1);
  Serial.print(" L0=");
  Serial.print(towar_left_0);
  Serial.print(" R0=");
  Serial.print(towar_right_0);
  Serial.print(" R1=");
  Serial.println(towar_right_1);
  updateMotors();
  motor_func();
}


void motor_func() {

  if (towar_left_1 == 0 && towar_left_0 == 1 && towar_right_0 == 1 && towar_right_1 == 0) {  //前进
    setAllMotors(LINE_FOLLOW_BASE_SPEED);
  }

  else if (towar_left_1 == 0 && towar_left_0 == 0 && towar_right_0 == 1 && towar_right_1 == 0) {  //偏向左，向右转
    setMotor(0, LIGHT_TURN_SPEED_FAST);
    setMotor(1, LIGHT_TURN_SPEED_SLOW);
    setMotor(2, LIGHT_TURN_SPEED_FAST);
    setMotor(3, LIGHT_TURN_SPEED_SLOW);
  }

  else if (towar_left_1 == 0 && towar_left_0 == 1 && towar_right_0 == 0 && towar_right_1 == 0) {  //向右偏，向左转
    setMotor(0, LIGHT_TURN_SPEED_SLOW);
    setMotor(1, LIGHT_TURN_SPEED_FAST);
    setMotor(2, LIGHT_TURN_SPEED_SLOW);
    setMotor(3, LIGHT_TURN_SPEED_FAST);
  }

  else if (towar_left_1 == 1 && towar_left_0 == 0 && towar_right_0 == 0 && towar_right_1 == 0) {

    setMotor(0, -HARD_TURN_SPEED_SLOW);
    setMotor(1, HARD_TURN_SPEED_FAST);
    setMotor(2, -HARD_TURN_SPEED_SLOW);
    setMotor(3, HARD_TURN_SPEED_FAST);

  }

  else if (towar_left_1 == 0 && towar_left_0 == 0 && towar_right_0 == 0 && towar_right_1 == 1) {

    setMotor(0, HARD_TURN_SPEED_FAST);
    setMotor(1, -HARD_TURN_SPEED_SLOW);
    setMotor(2, HARD_TURN_SPEED_FAST);
    setMotor(3, -HARD_TURN_SPEED_SLOW);

  }

  else if (towar_left_1 == 1 && towar_left_0 == 1 && towar_right_0 == 1 && towar_right_1 == 0) {

    setMotor(0, -LINE_FOLLOW_BASE_SPEED);
    setMotor(1, LINE_FOLLOW_BASE_SPEED);
    setMotor(2, -LINE_FOLLOW_BASE_SPEED);
    setMotor(3, LINE_FOLLOW_BASE_SPEED);

  }

  else if (towar_left_1 == 0 && towar_left_0 == 1 && towar_right_0 == 1 && towar_right_1 == 1) {

    setMotor(0, LINE_FOLLOW_BASE_SPEED);
    setMotor(1, -LINE_FOLLOW_BASE_SPEED);
    setMotor(2, LINE_FOLLOW_BASE_SPEED);
    setMotor(3, -LINE_FOLLOW_BASE_SPEED);
  }
}


void updateMotors() {
  if (millis() - lastUpdate < PWM_INTERVAL) return;
  lastUpdate = millis();

  for (int i = 0; i < PWM_CHANNELS; i++) {
    if (target_pwm[i] == current_pwm[i]) continue;

    int diff = target_pwm[i] - current_pwm[i];
    if (abs(diff) <= PWM_STEP) {
      current_pwm[i] = target_pwm[i];
    } else {
      current_pwm[i] += (diff > 0 ? PWM_STEP : -PWM_STEP);
    }
    applyMotorPWM(i, current_pwm[i]);
  }
}

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

void setMotor(int channel, int pwm_value) {
  pwm_value = constrain(pwm_value, -255, 255);
  target_pwm[channel] = pwm_value;
}

void setAllMotors(int pwm_value) {
  for (int i = 0; i < PWM_CHANNELS; i++) {
    setMotor(i, pwm_value);
  }
}
