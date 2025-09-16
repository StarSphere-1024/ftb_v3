#define ANALOG2_PIN_A GROVE3_PIN_A
#define ANALOG2_PIN_B GROVE3_PIN_B
#define ANALOG3_PIN_A GROVE6_PIN_A
#define ANALOG3_PIN_B GROVE6_PIN_B

#define SENSOR_L1 3  // 最左
#define SENSOR_L0 4  // 中左
#define SENSOR_R0 1  // 中右
#define SENSOR_R1 2  // 最右

void setup() {
    Serial.begin(9600);
    
    pinMode(SENSOR_L1, INPUT);
    pinMode(SENSOR_L0, INPUT);
    pinMode(SENSOR_R0, INPUT);
    pinMode(SENSOR_R1, INPUT);
}

void loop() {
    int l1 = analogRead(SENSOR_L1);
    int l0 = analogRead(SENSOR_L0);
    int r0 = analogRead(SENSOR_R0);
    int r1 = analogRead(SENSOR_R1);

    Serial.print("L1: ");
    Serial.print(l1);
    Serial.print("  L0: ");
    Serial.print(l0);
    Serial.print("  |  R0: ");
    Serial.print(r0);
    Serial.print("  R1: ");
    Serial.println(r1);

    delay(200);
}