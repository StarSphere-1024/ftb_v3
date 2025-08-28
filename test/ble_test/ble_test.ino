#include <NimBLEDevice.h>
#include <FastLED.h>
/*
- 服务器端：
  - 黄色常亮：正在广播，等待客户端连接。
  - 绿色常亮：客户端已连接。
- 客户端：
  - 蓝色常亮：正在扫描服务器。
  - 绿色常亮：已成功连接到服务器。
  - 红色常亮（持续1秒）：尝试连接但失败，之后会变回蓝色常亮重新扫描。
*/

// 定义角色（服务器或客户端）
// #define ROLE_SERVER // 取消注释以启用服务器模式；注释掉以启用客户端模式

#define RGB_PIN      33         // LED灯带数据引脚
#define NUM_RGB_LEDS 9          // LED数量
#define LED_TYPE     WS2812B
#define COLOR_ORDER  GRB
#define BRIGHTNESS   50         // 亮度 (0-255)
CRGB g_leds[NUM_RGB_LEDS];      // LED数组

enum LedState {
  STATE_ADVERTISING, // 广播中 (服务器)
  STATE_SCANNING,    // 扫描中 (客户端)
  STATE_CONNECTED,   // 已连接
  STATE_DISCONNECTED // 已断开
};
volatile LedState g_ledState;


void updateLedStatus() {
  switch (g_ledState) {
    case STATE_ADVERTISING: // 服务器广播中: 黄色常亮
      fill_solid(g_leds, NUM_RGB_LEDS, CRGB::Yellow); // -- 修改 --
      break;

    case STATE_SCANNING: // 客户端扫描中: 蓝色常亮
      fill_solid(g_leds, NUM_RGB_LEDS, CRGB::Blue); // -- 修改 --
      break;

    case STATE_CONNECTED: // 已连接: 绿色常亮 (此行无需修改)
      fill_solid(g_leds, NUM_RGB_LEDS, CRGB::Green);
      break;

    case STATE_DISCONNECTED: // 已断开: 红色常亮 (此行无需修改)
      fill_solid(g_leds, NUM_RGB_LEDS, CRGB::Red);
      break;
  }
}

// 公共变量和包含
#ifdef ROLE_SERVER
// 服务器特定变量
NimBLEServer* pServer;
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    g_ledState = STATE_CONNECTED;
    Serial.println("客户端已连接");
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    g_ledState = STATE_ADVERTISING;
    Serial.println("客户端断开连接 - 开始广播");
    NimBLEDevice::startAdvertising();
  }
};
ServerCallbacks serverCallbacks;

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    Serial.printf("特性 %s 被读取，值：%s\n",
                  pCharacteristic->getUUID().toString().c_str(),
                  pCharacteristic->getValue().c_str());
  }
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    Serial.printf("特性 %s 被写入，值：%s\n",
                  pCharacteristic->getUUID().toString().c_str(),
                  pCharacteristic->getValue().c_str());
  }
};
CharacteristicCallbacks chrCallbacks;
#else
// 客户端特定变量
NimBLEClient* pClient;
static const NimBLEAdvertisedDevice* advDevice;
static bool doConnect = false;
static uint32_t scanTimeMs = 5000;  // 扫描时间（毫秒）

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) override {
    g_ledState = STATE_CONNECTED;
    Serial.println("已连接到服务器");
  }
  void onDisconnect(NimBLEClient* pClient, int reason) override {
     g_ledState = STATE_SCANNING;
    Serial.printf("与服务器断开连接，原因 = %d - 开始扫描\n", reason);
    NimBLEDevice::getScan()->start(scanTimeMs);
  }
};
ClientCallbacks clientCallbacks;

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    Serial.printf("发现广播设备：%s\n", advertisedDevice->toString().c_str());
    if (advertisedDevice->isAdvertisingService(NimBLEUUID("DEAD"))) {
      Serial.println("找到目标服务");
      NimBLEDevice::getScan()->stop();
      advDevice = advertisedDevice;
      doConnect = true;
    }
  }
};
ScanCallbacks scanCallbacks;

// 通知回调
void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  Serial.printf("%s 来自 %s: 服务 = %s, 特性 = %s, 值 = %s\n",
                isNotify ? "通知" : "指示",
                pRemoteCharacteristic->getClient()->getPeerAddress().toString().c_str(),
                pRemoteCharacteristic->getRemoteService()->getUUID().toString().c_str(),
                pRemoteCharacteristic->getUUID().toString().c_str(),
                std::string((char*)pData, length).c_str());
  // Serial.flush();  // 确保输出完整
}
#endif

void setup() {
  Serial.begin(115200);
  Serial.println("启动BLE...");

  FastLED.addLeds<LED_TYPE, RGB_PIN, COLOR_ORDER>(g_leds, NUM_RGB_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  fill_solid(g_leds, NUM_RGB_LEDS, CRGB::Black); // 先熄灭所有灯
  FastLED.show();
#ifdef ROLE_SERVER
  // 服务器设置
  g_ledState = STATE_ADVERTISING;
  NimBLEDevice::init("NimBLE-Server");
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(&serverCallbacks);

  // 创建服务和特性
  NimBLEService* pService = pServer->createService("DEAD");
  NimBLECharacteristic* pCharacteristic = pService->createCharacteristic(
    "BEEF",
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  pCharacteristic->setValue("来自服务器的问候");
  pCharacteristic->setCallbacks(&chrCallbacks);
  pService->start();

  // 开始广播
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->start();
  Serial.println("服务器广播已开始");
#else
 g_ledState = STATE_SCANNING;
  // 客户端设置
  NimBLEDevice::init("NimBLE-Client");
  pClient = NimBLEDevice::createClient();
  pClient->setClientCallbacks(&clientCallbacks, false);

  // 开始扫描服务器
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(&scanCallbacks, false);
  pScan->setInterval(100);
  pScan->setWindow(100);
  pScan->setActiveScan(true);
  pScan->start(scanTimeMs);
  Serial.println("客户端开始扫描服务器");
#endif
}

void loop() {
  updateLedStatus();
  FastLED.show();
#ifdef ROLE_SERVER
  // 服务器循环
  delay(2000);
  if (pServer->getConnectedCount()) {
    NimBLEService* pSvc = pServer->getServiceByUUID("DEAD");
    if (pSvc) {
      NimBLECharacteristic* pChr = pSvc->getCharacteristic("BEEF");
      if (pChr) {
        pChr->notify();
      }
    }
  }
#else
  // 客户端循环
  delay(2000);
  if (doConnect) {
    doConnect = false;
    if (pClient->isConnected()) {
      Serial.println("已经连接，跳过连接尝试");
    } else {
      if (pClient->connect(advDevice)) {
        Serial.println("成功连接到服务器");
        NimBLERemoteService* pSvc = pClient->getService("DEAD");
        if (pSvc) {
          NimBLERemoteCharacteristic* pChr = pSvc->getCharacteristic("BEEF");
          if (pChr) {
            if (pChr->canRead()) {
              Serial.println("读取服务器值：" + String(pChr->readValue().c_str()));
            }
            if (pChr->canNotify()) {
              pChr->subscribe(true, notifyCB);
            }
          }
        }
      } else {
         g_ledState = STATE_DISCONNECTED;
        updateLedStatus();
        FastLED.show();
        delay(1000); // 短暂显示红色1秒钟
        g_ledState = STATE_SCANNING;
        Serial.println("连接失败，开始重新扫描");
        NimBLEDevice::getScan()->start(scanTimeMs);
      }
    }
  }
#endif
}