#define SERVO1_PIN 48
#define SERVO2_PIN 47
#define SERVO_TIMER_FREQUENCY 1000000
#define SERVO_TIMER_TICK_US 50
#define SERVO_PWM_PERIOD_MS 20
#define SERVO_PWM_PERIOD_TICKS (SERVO_PWM_PERIOD_MS * 1000 / SERVO_TIMER_TICK_US)
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
volatile uint16_t g_servo1_pulse_ticks, g_servo2_pulse_ticks;

hw_timer_t *g_servo_timer = NULL;
portMUX_TYPE g_servo_timer_mux = portMUX_INITIALIZER_UNLOCKED;

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

void initServo()
{
    pinMode(SERVO1_PIN, OUTPUT);
    pinMode(SERVO2_PIN, OUTPUT);
    g_servo_timer = timerBegin(SERVO_TIMER_FREQUENCY);
    timerAttachInterrupt(g_servo_timer, &on_servo_timer);
    timerAlarm(g_servo_timer, SERVO_TIMER_TICK_US, true, 0);
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

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    delay(1000);
    Serial.println("\n\nESP32-S3 Servo Test Program Starting...");

    initServo();

    Servos_SetAngle(1, 0);
    Servos_SetAngle(2, 0);

    Serial.println("Servo test setup complete.");
}

void loop()
{
    delay(1000);
}