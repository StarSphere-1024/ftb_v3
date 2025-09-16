#include <PS2X_lib.h>

#define PS2_CMD_PIN 9
#define PS2_DATA_PIN 10
#define PS2_CLK_PIN 41
#define PS2_CS_PIN 42

PS2X ps2x;
int ps2_error = 0;
byte ps2_type = 0;
unsigned long lastUpdate = 0;
int buttonCount = 16;
struct ButtonMap
{
    uint16_t button;
    const char *name;
};
const ButtonMap buttons[] = {
    {PSB_PAD_UP, "Up"},
    {PSB_PAD_RIGHT, "Right"},
    {PSB_PAD_DOWN, "Down"},
    {PSB_PAD_LEFT, "Left"},
    {PSB_SELECT, "Select"},
    {PSB_START, "Start"},
    {PSB_L1, "L1"},
    {PSB_R1, "R1"},
    {PSB_L2, "L2"},
    {PSB_R2, "R2"},
    {PSB_TRIANGLE, "Triangle"},
    {PSB_CIRCLE, "Circle"},
    {PSB_CROSS, "Cross"},
    {PSB_SQUARE, "Square"},
    {PSB_L3, "L3"},
    {PSB_R3, "R3"}};

void setup()
{

    Serial.begin(115200);

    ps2_error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_CS_PIN, PS2_DATA_PIN, true, true);
    if (ps2_error == 0)
        Serial.println("PS2手柄配置成功。");
    else
    {
        Serial.print("PS2手柄配置失败，错误码: ");
        Serial.println(ps2_error);
    }

    buttonCount = sizeof(buttons) / sizeof(buttons[0]);
    lastUpdate = millis();
}

void loop()
{

    ps2x.read_gamepad(false, false);

    for (int i = 0; i < buttonCount; i++)
    {
        if (ps2x.Button(buttons[i].button))
        {
            Serial.println(buttons[i].name);
        }
    }

    int stick_lx = ps2x.Analog(PSS_LX);
    int stick_ly = ps2x.Analog(PSS_LY);
    if (stick_lx != 128 || stick_ly != 128)
    {
        Serial.printf("Left Stick: (%d, %d)\n", stick_lx, stick_ly);
    }
    int stick_rx = ps2x.Analog(PSS_RX);
    int stick_ry = ps2x.Analog(PSS_RY);
    if (stick_rx != 128 || stick_ry != 128)
    {
        Serial.printf("Right Stick: (%d, %d)\n", stick_rx, stick_ry);
    }

    if (millis() - lastUpdate >= 50)
    {
        lastUpdate = millis();
    }
    else
    {
        continue;
    }
}
