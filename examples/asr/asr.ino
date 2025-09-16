#include <Arduino.h>
#include <FastLED.h>

#define RGB_PIN 33
#define NUM_RGB_LEDS 9
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define INITIAL_RGB_BRIGHTNESS 10
CRGB g_leds[NUM_RGB_LEDS];

#define ASR_RX_PIN 35
#define ASR_TX_PIN 34

bool rgb_on = true;
uint8_t hue = 0;

void setup()
{
    Serial.begin(9600);
    Serial2.begin(115200, SERIAL_8N1, ASR_RX_PIN, ASR_TX_PIN);

    Serial2.println("SetVolume:6"); // 设置ASR音量为6

    FastLED.addLeds<LED_TYPE, RGB_PIN, COLOR_ORDER>(g_leds, NUM_RGB_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(INITIAL_RGB_BRIGHTNESS);
    fill_solid(g_leds, NUM_RGB_LEDS, CRGB::Black);
    FastLED.show();
    Serial.println("RGB LEDs initialized.");
}

void loop()
{
    // 语音串口控制RGB开关
    if (Serial2.available() > 0) {
        String command = Serial2.readStringUntil('\n');
        command.trim();
        if (command.equalsIgnoreCase("RGB:ON")) {
            rgb_on = true;
        } else if (command.equalsIgnoreCase("RGB:OFF")) {
            rgb_on = false;
            fill_solid(g_leds, NUM_RGB_LEDS, CRGB::Black);
            FastLED.show();
        }
    }

    // 彩虹流动光效
    if (rgb_on) {
        fill_rainbow(g_leds, NUM_RGB_LEDS, hue++, 7);
        FastLED.show();
    }
    delay(20);
}
