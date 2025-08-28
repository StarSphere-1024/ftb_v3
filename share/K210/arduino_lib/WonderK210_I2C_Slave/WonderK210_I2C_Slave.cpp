// WonderK210_I2C.cpp
#include "WonderK210_I2C.h"

// 构造函数
WonderK210_I2C::WonderK210_I2C(TwoWire *wire) : _wire(wire)
{
    _data_updated = false;
    // 清空内部数据
    memset(&_box_data, 0, sizeof(Find_Box_st));
}

// 初始化函数
void WonderK210_I2C::begin(int sda_pin, int scl_pin, uint32_t frequency)
{
    if (sda_pin >= 0 && scl_pin >= 0) {
        _wire->begin(sda_pin, scl_pin, frequency);
    } else {
        _wire->begin(); // 使用默认引脚
    }
    _wire->setClock(frequency);
    Serial.println("WonderK210 I2C Master initialized");
}

// 更新数据函数
bool WonderK210_I2C::update_data()
{
    // 从 K210_I2C_ADDR 设备请求 sizeof(Find_Box_st) 个字节的数据
    // sizeof(Find_Box_st) 应该是 8 (4 * uint16_t)
    uint8_t bytes_received = _wire->requestFrom(K210_I2C_ADDR, (uint8_t)sizeof(Find_Box_st));

    // 检查是否收到了正确数量的字节
    if (bytes_received == sizeof(Find_Box_st))
    {
        // 创建一个缓冲区来接收数据
        uint8_t buffer[sizeof(Find_Box_st)];
        _wire->readBytes(buffer, sizeof(Find_Box_st));

        // 将接收到的字节流直接拷贝到结构体中
        // C++保证了当结构体被 #pragma pack(1) 修饰时，这种操作是安全的
        // MicroPython的ustruct默认使用小端模式，大部分ESP32也是小端模式，所以直接拷贝是可行的
        memcpy(&_box_data, buffer, sizeof(Find_Box_st));

        _data_updated = true;
        return true;
    }
    else
    {
        // 如果没有收到预期的数据，则认为更新失败
        Serial.printf("Error: Expected %d bytes, but received %d\n", sizeof(Find_Box_st), bytes_received);
        _data_updated = false;
        return false;
    }
}

// 获取方框数据函数
bool WonderK210_I2C::get_box(Find_Box_st *rec)
{
    if (_data_updated)
    {
        // 将内部存储的数据拷贝给用户提供的结构体指针
        memcpy(rec, &_box_data, sizeof(Find_Box_st));
        return true;
    }
    return false;
}