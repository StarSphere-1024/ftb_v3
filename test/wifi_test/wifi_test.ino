
#include <WiFi.h>
#include <FastLED.h>

/*
 * AP模式：
 *  黄色常亮：AP已启动，正在等待客户端连接。
 *  绿色常亮：有客户端成功连接。
 * STA模式：
 *  蓝色常亮：正在扫描并尝试连接AP。
 *  绿色常亮：已成功连接到AP。
 *  红色常亮：连接断开或在指定时间内未能连接成功
 */

// --- 角色定义 ---
// 取消注释以启用AP模式；注释掉则为STA（客户端）模式
// #define ROLE_AP

// --- WiFi凭证 (STA模式需要连接的目标AP) ---
const char *ssid = "ESP32-AP-Test";
const char *password = "12345678";

// --- RGB LED 定义 ---
#define RGB_PIN 33     // LED灯带数据引脚 (请根据您的ESP32-S3板子修改)
#define NUM_RGB_LEDS 9 // LED数量
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS 50 // 亮度 (0-255)

CRGB g_leds[NUM_RGB_LEDS];

// 定义LED状态
enum LedState
{
    STATE_AP_WAITING,      // AP模式: 等待连接
    STATE_AP_CONNECTED,    // AP模式: 客户端已连接
    STATE_STA_SCANNING,    // STA模式: 扫描和连接中
    STATE_STA_CONNECTED,   // STA模式: 已连接
    STATE_STA_DISCONNECTED // STA模式: 连接断开
};
volatile LedState g_ledState; // 当前LED状态

unsigned long lastWifiCheck = 0; // 用于非阻塞检查WiFi状态的计时器

// LED更新函数
void updateLedStatus() {
  switch (g_ledState) {
    case STATE_AP_WAITING: // AP等待连接: 黄色常亮
      fill_solid(g_leds, NUM_RGB_LEDS, CRGB::Yellow); 
      break;

    case STATE_AP_CONNECTED: // AP已连接: 绿色常亮 
      fill_solid(g_leds, NUM_RGB_LEDS, CRGB::Green);
      break;

    case STATE_STA_SCANNING: // STA扫描中: 蓝色常亮
      fill_solid(g_leds, NUM_RGB_LEDS, CRGB::Blue); 
      break;
    
    case STATE_STA_CONNECTED: // STA已连接: 根据RSSI显示绿色灯条
      {
        long rssi = WiFi.RSSI();
        int ledsToShow = map(constrain(rssi, -90, -30), -90, -30, 1, NUM_RGB_LEDS);
        fill_solid(g_leds, NUM_RGB_LEDS, CRGB::Black);
        fill_solid(g_leds, ledsToShow, CRGB::Green);
      }
      break;

    case STATE_STA_DISCONNECTED: // STA已断开: 红色常亮
      fill_solid(g_leds, NUM_RGB_LEDS, CRGB::Red);
      break;
  }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("\nStarting WiFi Distance Test...");

    // 初始化FastLED
    FastLED.addLeds<LED_TYPE, RGB_PIN, COLOR_ORDER>(g_leds, NUM_RGB_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);
    fill_solid(g_leds, NUM_RGB_LEDS, CRGB::Black);
    FastLED.show();
    delay(500);

#ifdef ROLE_AP
    // AP (服务器) 模式设置
    Serial.println("Role: AP Server");
    g_ledState = STATE_AP_WAITING;
    WiFi.softAP(ssid, password);
    IPAddress apIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(apIP);
#else
    // STA (客户端) 模式设置
    Serial.println("Role: STA Client");
    g_ledState = STATE_STA_SCANNING;
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to ");
    Serial.println(ssid);
#endif
}

void loop()
{
    // 每200毫秒检查一次WiFi状态，避免过于频繁
    if (millis() - lastWifiCheck > 200)
    {
#ifdef ROLE_AP
        // AP模式逻辑
        if (WiFi.softAPgetStationNum() > 0)
        {
            if (g_ledState != STATE_AP_CONNECTED)
            {
                Serial.println("Client Connected!");
                g_ledState = STATE_AP_CONNECTED;
            }
        }
        else
        {
            if (g_ledState != STATE_AP_WAITING)
            {
                Serial.println("Client Disconnected. Waiting for new connection...");
                g_ledState = STATE_AP_WAITING;
            }
        }
#else
        // STA模式逻辑
        if (WiFi.status() == WL_CONNECTED)
        {
            if (g_ledState != STATE_STA_CONNECTED)
            {
                Serial.println("\nWiFi Connected!");
                Serial.print("IP Address: ");
                Serial.println(WiFi.localIP());
                g_ledState = STATE_STA_CONNECTED;
            }
        }
        else
        {
            if (g_ledState != STATE_STA_SCANNING)
            {
                // 如果之前是连接状态，现在断开了，就显示红色
                if (g_ledState == STATE_STA_CONNECTED)
                {
                    Serial.println("WiFi Disconnected! Reconnecting...");
                    WiFi.reconnect();
                }
                g_ledState = STATE_STA_SCANNING;
            }
        }
#endif
        lastWifiCheck = millis();
    }

    // 持续更新LED状态并显示
    updateLedStatus();
    FastLED.show();

    delay(10);
}