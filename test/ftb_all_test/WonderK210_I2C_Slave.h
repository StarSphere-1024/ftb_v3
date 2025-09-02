#ifndef WONDERK210_I2C_H
#define WONDERK210_I2C_H

#include <Arduino.h>
#include <Wire.h>

#define K210_I2C_ADDR 0x32
#define K210_DATA_REG 0x01

const uint16_t I2C_BUFFER_SIZE = 40;
const uint16_t I2C_DATA_SIZE = 30;
const uint16_t I2C_MSG_SIZE = 20;

#pragma pack(1)

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
    char msg[I2C_MSG_SIZE];
} Find_Box_Msg_st;

typedef struct
{
    char msg[I2C_MSG_SIZE];
} Find_Msg_st;

#pragma pack()

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
    WonderK210_I2C(TwoWire *wire = &Wire, uint8_t deviceAddress = K210_I2C_ADDR);
    void begin(int sda = -1, int scl = -1);

    bool update_data();
    bool recive_box(Find_Box_st *rec, enum k210_PACKET_FUNCTION func);
    bool recive_box_msg(Find_Box_Msg_st *rec, enum k210_PACKET_FUNCTION func);
    bool recive_msg(Find_Msg_st *rec, enum k210_PACKET_FUNCTION func);

private:
    TwoWire *_wire;
    uint8_t _address;

    bool _read_succeed;
    uint8_t _packet_function;
    uint8_t _packet_length;
    uint8_t _packet_data[I2C_DATA_SIZE];
    
    static const uint8_t crc8_table[];
    static uint8_t checksum_crc8(const uint8_t *buf, uint16_t len);
};

#endif