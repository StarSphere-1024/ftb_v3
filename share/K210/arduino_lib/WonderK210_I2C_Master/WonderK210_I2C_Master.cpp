// WonderK210_I2C.cpp
#include "WonderK210_I2C.h"

// CRC8 字节表 (与原版相同)
static const uint8_t crc8_table[] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

// CRC校验 (与原版相同)
static uint16_t checksum_crc8(const uint8_t *buf, uint16_t len)
{
    uint8_t check = 0;
    while (len--)
    {
        check = crc8_table[check ^ (*buf++)];
    }
    return ((uint16_t)check) & 0x00FF;
}

// 创建一个静态实例指针，用于在静态中断服务函数中访问类成员
static WonderK210_I2C* instance = nullptr;

// 构造函数
WonderK210_I2C::WonderK210_I2C(uint8_t slave_address) : _slave_address(slave_address)
{
    pk_ctl.index_head = 0;
    pk_ctl.index_tail = 0;
    pk_ctl.len = 0;
    pk_ctl.state = K210_CONTROLLER_STATE_STARTBYTE1;
    read_succeed = false;
    instance = this; // 将当前对象实例赋值给静态指针
}

// 初始化函数
void WonderK210_I2C::begin()
{
    Wire.begin(_slave_address); // 作为从机加入I2C总线
    Wire.onReceive(receiveEvent); // 注册接收事件的回调函数
    Serial.println("WonderK210 I2C Slave initialized");
}

// I2C 接收事件回调函数 (ISR)
// 当主机发送数据时，此函数会被自动调用
void WonderK210_I2C::receiveEvent(int howMany)
{
    if (instance == nullptr) return; // 如果实例不存在，则返回

    // 将从I2C缓冲区接收到的数据存入我们自己的环形缓冲区
    while (Wire.available())
    {
        instance->pk_ctl.data[instance->pk_ctl.index_tail] = (uint8_t)Wire.read();
        instance->pk_ctl.index_tail++;
        if (BUFFER_SIZE <= instance->pk_ctl.index_tail)
        {
            instance->pk_ctl.index_tail = 0;
        }
        
        // 环形缓冲区溢出处理
        if (instance->pk_ctl.index_tail == instance->pk_ctl.index_head)
        {
            instance->pk_ctl.index_head++;
            if (BUFFER_SIZE <= instance->pk_ctl.index_head)
            {
                instance->pk_ctl.index_head = 0;
            }
        }
        else
        {
            instance->pk_ctl.len++;
        }
    }
}

// 更新解析函数
// 此函数应在主循环 loop() 中被频繁调用
void WonderK210_I2C::update_data()
{
    // 数据接收部分已由 receiveEvent 中断处理
    // 这里只负责解析环形缓冲区中的数据
    uint8_t crc = 0;
    while (pk_ctl.len > 0)
    {
        // 状态机解析逻辑 (与原版完全相同)
        switch (pk_ctl.state)
        {
        case K210_CONTROLLER_STATE_STARTBYTE1:
            pk_ctl.state = K210_CONST_STARTBYTE1 == pk_ctl.data[pk_ctl.index_head] ? K210_CONTROLLER_STATE_STARTBYTE2 : K210_CONTROLLER_STATE_STARTBYTE1;
            break;
        case K210_CONTROLLER_STATE_STARTBYTE2:
            pk_ctl.state = K210_CONST_STARTBYTE2 == pk_ctl.data[pk_ctl.index_head] ? K210_CONTROLLER_STATE_FUNCTION : K210_CONTROLLER_STATE_STARTBYTE1;
            break;
        case K210_CONTROLLER_STATE_FUNCTION:
            pk_ctl.state = K210_FUNC_NONE > pk_ctl.data[pk_ctl.index_head] ? K210_CONTROLLER_STATE_LENGTH : K210_CONTROLLER_STATE_STARTBYTE1;
            if (K210_CONTROLLER_STATE_LENGTH == pk_ctl.state)
            {
                pk_ctl.frame.function = pk_ctl.data[pk_ctl.index_head];
            }
            break;
        case K210_CONTROLLER_STATE_LENGTH:
            if (pk_ctl.data[pk_ctl.index_head] >= DATA_SIZE)
            {
                pk_ctl.state = K210_CONTROLLER_STATE_STARTBYTE1;
                // 注意：这里原版用了 continue，但在循环外层，应使用 break 并处理索引
            }
            else
            {
                pk_ctl.frame.data_length = pk_ctl.data[pk_ctl.index_head];
                pk_ctl.state = (0 == pk_ctl.frame.data_length) ? K210_CONTROLLER_STATE_CHECKSUM : K210_CONTROLLER_STATE_DATA;
                pk_ctl.data_index = 0;
            }
            break;
        case K210_CONTROLLER_STATE_DATA:
            pk_ctl.frame.data[pk_ctl.data_index] = pk_ctl.data[pk_ctl.index_head];
            ++pk_ctl.data_index;
            if (pk_ctl.data_index >= pk_ctl.frame.data_length)
            {
                pk_ctl.state = K210_CONTROLLER_STATE_CHECKSUM;
                pk_ctl.frame.data[pk_ctl.data_index] = '\0';
            }
            break;
        case K210_CONTROLLER_STATE_CHECKSUM:
            pk_ctl.frame.checksum = pk_ctl.data[pk_ctl.index_head];
            crc = checksum_crc8((uint8_t *)&pk_ctl.frame.function, pk_ctl.frame.data_length + 2);
            if (crc == pk_ctl.frame.checksum)
            {
                memcpy(&pk_result, &pk_ctl.frame, sizeof(K210_packet_st));
                pk_result.start_byte1 = K210_CONST_STARTBYTE1;
                pk_result.start_byte2 = K210_CONST_STARTBYTE2;
                read_succeed = true;
            }
            memset(&pk_ctl.frame, 0, sizeof(struct k210_PacketRawFrame));
            pk_ctl.state = K210_CONTROLLER_STATE_STARTBYTE1;
            break;
        default:
            pk_ctl.state = K210_CONTROLLER_STATE_STARTBYTE1;
            break;
        }

        // 移动环形缓冲区的头指针
        pk_ctl.index_head++;
        if (BUFFER_SIZE <= pk_ctl.index_head)
        {
            pk_ctl.index_head = 0;
        }
        pk_ctl.len--;
    }
}

void WonderK210_I2C::clear_rec()
{
    // 清空内部环形缓冲区
    pk_ctl.index_head = pk_ctl.index_tail;
    pk_ctl.len = 0;
    read_succeed = false;
    memset(&pk_result, 0, sizeof(K210_packet_st));
}

// 以下用户函数与原版完全相同
bool WonderK210_I2C::recive_box(Find_Box_st *rec, enum k210_PACKET_FUNCTION func)
{
    if (false == read_succeed) return false;
    if (func == pk_result.function)
    {
        read_succeed = false;
        Find_Box_st *temp = (Find_Box_st *)pk_result.data;
        rec->x = temp->x;
        rec->y = temp->y;
        rec->w = temp->w;
        rec->h = temp->h;
        return true;
    }
    return false;
}

bool WonderK210_I2C::recive_box_msg(Find_Box_Msg_st *rec, enum k210_PACKET_FUNCTION func)
{
    if (false == read_succeed) return false;
    if (func == pk_result.function)
    {
        read_succeed = false;
        Find_Box_Msg_st *temp = (Find_Box_Msg_st *)pk_result.data;
        rec->x = temp->x;
        rec->y = temp->y;
        rec->w = temp->w;
        rec->h = temp->h;
        strcpy(rec->msg, temp->msg);
        return true;
    }
    return false;
}

bool WonderK210_I2C::recive_msg(Find_Msg_st *rec, enum k210_PACKET_FUNCTION func)
{
    if (false == read_succeed) return false;
    if (func == pk_result.function)
    {
        read_succeed = false;
        Find_Msg_st *temp = (Find_Msg_st *)pk_result.data;
        strcpy(rec->msg, temp->msg);
        return true;
    }
    return false;
}