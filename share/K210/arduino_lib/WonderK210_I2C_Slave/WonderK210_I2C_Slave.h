// WonderK210_I2C.h
#ifndef WONDERK210_I2C_H
#define WONDERK210_I2C_H

#include <Arduino.h>
#include <Wire.h>

#define K210_I2C_ADDR 0x24 // K210 I2C 从机地址

// 关闭字节对齐，确保结构体大小准确
#pragma pack(1)

// 方框数据结构体
typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
} Find_Box_st;

// (为了兼容性保留，但在当前I2C实现中未使用)
typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    char msg[20];
} Find_Box_Msg_st;

// (为了兼容性保留，但在当前I2C实现中未使用)
typedef struct
{
    char msg[20];
} Find_Msg_st;

#pragma pack()


class WonderK210_I2C
{
public:
    // 构造函数
    WonderK210_I2C(TwoWire *wire = &Wire);

    // 初始化I2C
    void begin(int sda_pin = -1, int scl_pin = -1, uint32_t frequency = 100000);

    // 从K210请求并更新数据
    bool update_data();

    // 获取方框数据
    // 注意：这个函数现在只是返回最后一次 update_data() 获取的数据
    bool get_box(Find_Box_st *rec);


private:
    TwoWire *_wire; // I2C总线对象
    Find_Box_st _box_data; // 内部存储的数据
    bool _data_updated; // 数据是否成功更新的标志
};

#endif // WONDERK210_I2C_H