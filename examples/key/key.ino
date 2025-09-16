
// 引入 Arduino 和 FastLED 库
#include <Arduino.h>
#include <FastLED.h>

// RGB 灯带相关定义
#define RGB_PIN 33              // RGB 灯带数据引脚
#define NUM_RGB_LEDS 9          // 灯珠数量
#define RGB_BRIGHTNESS 10       // 灯带亮度
#define LED_TYPE WS2812B        // 灯带类型
#define COLOR_ORDER GRB         // 颜色顺序
CRGB leds[NUM_RGB_LEDS];      // 灯带对象

// 用户按键引脚定义
#define USER_BUTTON_A_PIN 21    // 按键A引脚
#define USER_BUTTON_B_PIN 0     // 按键B引脚

// 初始化串口、按键和灯带
void setup()
{
    Serial.begin(9600); // 初始化串口
    pinMode(USER_BUTTON_A_PIN, INPUT_PULLUP); // 设置按键A为上拉输入
    pinMode(USER_BUTTON_B_PIN, INPUT_PULLUP); // 设置按键B为上拉输入
    Serial.println("Key setup complete.");
    FastLED.addLeds<LED_TYPE, RGB_PIN, COLOR_ORDER>(leds, NUM_RGB_LEDS).setCorrection(TypicalLEDStrip); // 初始化灯带
    FastLED.setBrightness(RGB_BRIGHTNESS); // 设置灯带亮度
    fill_solid(leds, NUM_RGB_LEDS, CRGB::Black); // 灯带全灭
    FastLED.show();
}

// 主循环，检测按键并控制灯带
void loop()
{
    // 检测按键A是否被按下
    if (digitalRead(USER_BUTTON_A_PIN) == LOW)
    {
        fill_solid(leds, NUM_RGB_LEDS, CRGB::Red); // 灯带变红
        FastLED.show();
        Serial.println("用户按键 A 被按下!");
        while (digitalRead(USER_BUTTON_A_PIN) == LOW) // 等待按键释放
            ;
        fill_solid(leds, NUM_RGB_LEDS, CRGB::Black); // 灯带熄灭
        FastLED.show();
    }
    // 检测按键B是否被按下
    if (digitalRead(USER_BUTTON_B_PIN) == LOW)
    {
        fill_solid(leds, NUM_RGB_LEDS, CRGB::Green); // 灯带变绿
        FastLED.show();
        Serial.println("用户按键 B 被按下!");
        while (digitalRead(USER_BUTTON_B_PIN) == LOW) // 等待按键释放
            ;
        fill_solid(leds, NUM_RGB_LEDS, CRGB::Black); // 灯带熄灭
        FastLED.show();
    }
    delay(20); // 延时，防止抖动
}
