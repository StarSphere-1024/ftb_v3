# find_face_yolo_i2c.py
'''
@file:    find_face_yolo_i2c.py
@company: Hiwonder
@author:  CuZn & Gemini
@date:    2025-08-26
@description: 人脸检测，并将识别后的数据通过 I2C 发送给 ESP32-S3 开发板。
        # K210视觉模块 (I2C Master) 会将识别到的方框数据和识别信息 发送给 开发板 (I2C Slave)

'''

# 载入相关模块
import sensor, image, time, lcd
from maix import KPU
from machine import I2C
import binascii

# ================= K210 I2C Master 配置 =================
# ESP32-S3 的 I2C 从机地址
SLAVE_ADDR = 0x24

# K210 I2C 引脚配置
SCL_PIN = 18
SDA_PIN = 19
I2C_BUS_ID = I2C.I2C3

# 初始化I2C为 Master 模式
# 频率可以设置为 100000 (100K) 或 400000 (400K)
i2c = I2C(I2C_BUS_ID, mode=I2C.MODE_MASTER, scl=SCL_PIN, sda=SDA_PIN, freq=400000)
print("I2C Master initialized on Bus {} with SCL:{} SDA:{}".format(I2C_BUS_ID, SCL_PIN, SDA_PIN))
# =======================================================


##################################### send_func begin #####################################

# CRC校验参数，保持不变
crc8_table = [
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
]

# CRC校验函数
def checksum_crc8(data):
    check = 0
    for b in data:
        check = crc8_table[check ^ b]
    return check & 0x00FF

# 功能号
class FuncTag:
    START = 0
    FIND_FACE_YOLO  = 5

# 字符串转int (此函数在原代码中存在，但似乎未被正确使用，这里保留)
def str_2_int(data_str):
    bb = binascii.hexlify(data_str)
    bb = str(bb)[2:-1]
    int_h = int(bb[0])*16
    int_l = int(bb[1],16)
    return int_h+int_l

def send_data(x, y, w, h, msg):
    '''
    数据包格式: 0xAA 0x55 功能号 数据长度 data CRC
    通过 I2C 发送
    '''
    START_1 = 0xAA
    START_2 = 0x55
    FUNC_num = FuncTag.FIND_FACE_YOLO # 功能编号
    data = [] # 数据组

    # 如果有坐标数据，则打包
    if not (x==0 and y==0 and w==0 and h==0):
        # x(小端模式)
        data.append(x & 0xFF)
        data.append((x >> 8) & 0xFF)
        # y(小端模式)
        data.append(y & 0xFF)
        data.append((y >> 8) & 0xFF)
        # w(小端模式)
        data.append(w & 0xFF)
        data.append((w >> 8) & 0xFF)
        # h(小端模式)
        data.append(h & 0xFF)
        data.append((h >> 8) & 0xFF)

    # 如果有消息数据，则打包
    if msg is not None:
        msg_bytes = msg.encode('utf-8')
        for byte in msg_bytes:
            data.append(byte)

    Length = len(data)
    # 准备校验计算的数据包 = 功能号 + 长度 + 数据
    crc_buf = [FUNC_num, Length] + data
    crc = checksum_crc8(crc_buf)

    # 准备最终发送的数据包
    send_buf = [START_1, START_2] + crc_buf + [crc]

    try:
        # 将列表转换为 bytearray 并通过 I2C 发送
        i2c.writeto(SLAVE_ADDR, bytearray(send_buf))
        print("I2C Sent:", [hex(b) for b in send_buf]) # 打印十六进制发送内容
    except Exception as e:
        print("I2C Write Error:", e)

##################################### send_func end #####################################

# 定义发送数据
send_x = 0
send_y = 0
send_w = 0
send_h = 0
send_msg = ""

# 初始化LCD和传感器等... (与原代码相同)
lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 100)

clock = time.clock()
img_obj = image.Image(size=(320,256))
anchor = (0.893, 1.463, 0.245, 0.389, 1.55, 2.58, 0.375, 0.594, 3.099, 5.038, 0.057, 0.090, 0.567, 0.904, 0.101, 0.160, 0.159, 0.255)
kpu = KPU()
kpu.load_kmodel("/sd/KPU/yolo_face_detect/yolo_face_detect.kmodel")
kpu.init_yolo2(anchor, anchor_num=9, img_w=320, img_h=240, net_w=320, net_h=256, layer_w=10, layer_h=8, threshold=0.7, nms_value=0.3, classes=1)


try:
    while True:
        clock.tick()
        img = sensor.snapshot()
        img_obj.draw_image(img, 0,0)
        img_obj.pix_to_ai()
        kpu.run_with_output(img_obj)
        dect = kpu.regionlayer_yolo2()
        fps = clock.fps()

        found_face = False
        if len(dect) > 0:
            found_face = True
            for l in dect:
                img.draw_rectangle(l[0],l[1],l[2],l[3], color=(0, 255, 0))
                # 将第一个检测到的人脸数据用于发送
                if found_face:
                    send_x = l[0]
                    send_y = l[1]
                    send_w = l[2]
                    send_h = l[3]
                    found_face = False # 只发送第一个
            # 发送数据
            send_data(send_x, send_y, send_w, send_h, None)
        else:
            # 如果没有检测到人脸，可以发送一个空包或者不发送
            # send_data(0, 0, 0, 0, None) # 可选：发送空数据包
            pass

        img.draw_string(0, 0, "%2.1ffps" %(fps), color=(0, 60, 128), scale=2.0)
        lcd.display(img)

except Exception as e:
    raise e
finally:
    kpu.deinit()
