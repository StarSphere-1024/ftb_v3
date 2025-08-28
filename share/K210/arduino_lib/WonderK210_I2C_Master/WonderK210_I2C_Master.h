// WonderK210_I2C.h
#ifndef WONDERK210_I2C_H
#define WONDERK210_I2C_H

#include <Arduino.h>
#include <Wire.h>

#define K210_CONST_STARTBYTE1 0xAAu
#define K210_CONST_STARTBYTE2 0x55u
const uint16_t BUFFER_SIZE = 90;
const uint16_t DATA_SIZE = 30; // 有效数据不应该超过 30
const uint16_t MSG_SIZE = 20;

// 解析器状态机状态枚举 (与原版相同)
enum k210_PacketControllerState
{
    K210_CONTROLLER_STATE_STARTBYTE1,
    K210_CONTROLLER_STATE_STARTBYTE2,
    K210_CONTROLLER_STATE_FUNCTION,
    K210_CONTROLLER_STATE_LENGTH,
    K210_CONTROLLER_STATE_DATA,
    K210_CONTROLLER_STATE_CHECKSUM,
};

#pragma pack(1)
// 命令内容参数 (与原版相同)
typedef struct k210_PacketRawFrame
{
    uint8_t start_byte1;
    uint8_t start_byte2;
    uint8_t function;
    uint8_t data_length;
    uint8_t data[DATA_SIZE];
    uint8_t checksum;
} K210_packet_st;
#pragma pack(0)

// 协议解析器 (与原版相同)
struct k210_PacketController
{
    enum k210_PacketControllerState state;
    struct k210_PacketRawFrame frame;
    int data_index;
    uint8_t len;
    uint8_t index_head;
    uint8_t index_tail;
    uint8_t data[BUFFER_SIZE];
};

#pragma pack(1)
// 数据结构体 (与原版相同)
typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
} Find_Box_st;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    char msg[MSG_SIZE];
} Find_Box_Msg_st;

typedef struct
{
    char msg[MSG_SIZE];
} Find_Msg_st;
#pragma pack()

// 帧功能号枚举 (与原版相同)
enum k210_PACKET_FUNCTION
{
    K210_FUNC_SYS = 0,
    K210_COLOR_RECOGNITION,
    K210_FIND_BARCODES,
    K210_FIND_QRCODES,
    K210_FIND_APRILTAGS,
    K210_FIND_FACE_YOLO,
    K210_FIND_FACEFEATURE,
    K210_FIND_FACEMASK,
    K210_FIND_OBJECT,
    K210_FIND_SELF_LEARNING,
    K210_FIND_DIGITAL,
    K210_FIND_FACE_RECOGNITION,
    K210_FIND_RED_FOLLOW,
    K210_FIND_SIGNPOST_FOLLOW,
    K210_FIND_DIGITAL_CARD,
    K210_GARBAGE_SORTING,
    K210_COLOR_SORTING,
    K210_FUNC_NONE
};

class WonderK210_I2C
{
public:
    // 构造函数，接受I2C从机地址
    WonderK210_I2C(uint8_t slave_address);
    // 初始化函数
    void begin();
    // 更新解析函数
    void update_data();
    // 清空内部缓冲区
    void clear_rec();

    // 用户使用函数 (与原版相同)
    bool recive_box(Find_Box_st *rec, enum k210_PACKET_FUNCTION func);
    bool recive_box_msg(Find_Box_Msg_st *rec, enum k210_PACKET_FUNCTION func);
    bool recive_msg(Find_Msg_st *rec, enum k210_PACKET_FUNCTION func);

    // I2C接收中断处理函数，必须为 public 和 static
    static void receiveEvent(int howMany);

private:
    uint8_t _slave_address;
    
    // 读取成功标志位
    bool read_succeed;
    // 接收解析控制器
    struct k210_PacketController pk_ctl;
    // 解析结果储存变量
    K210_packet_st pk_result;
};

#endif // WONDERK210_I2C_H