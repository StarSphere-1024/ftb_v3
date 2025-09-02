import sensor, image, time, lcd
from maix import KPU
import hiwonder

i2c = hiwonder.hw_slavei2c()

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

def checksum_crc8(data):
    check = 0
    for b in data:
        check = crc8_table[check ^ b]
    return check & 0x00FF

MAX_PACKET_SIZE = 33
i2c_buffer = bytearray([0] * MAX_PACKET_SIZE)

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

K210_FIND_FACE_YOLO = 5

try:
    while True:
        clock.tick()
        img = sensor.snapshot()
        img_obj.draw_image(img, 0,0)
        img_obj.pix_to_ai()
        kpu.run_with_output(img_obj)
        dect = kpu.regionlayer_yolo2()
        fps = clock.fps()

        face_detected = False
        if len(dect) > 0:
            face_detected = True
            l = dect[0]
            img.draw_rectangle(l[0],l[1],l[2],l[3], color=(0, 255, 0))

            func = K210_FIND_FACE_YOLO
            data_len = 8
            data = bytearray(8)
            data[0] = l[0] & 0xFF
            data[1] = (l[0] >> 8) & 0xFF
            data[2] = l[1] & 0xFF
            data[3] = (l[1] >> 8) & 0xFF
            data[4] = l[2] & 0xFF
            data[5] = (l[2] >> 8) & 0xFF
            data[6] = l[3] & 0xFF
            data[7] = (l[3] >> 8) & 0xFF

            checksum_payload = bytearray([func, data_len]) + data
            crc = checksum_crc8(checksum_payload)

            i2c_buffer[0] = func
            i2c_buffer[1] = data_len
            i2c_buffer[2:10] = data
            i2c_buffer[10] = crc

        if not face_detected:
            for i in range(MAX_PACKET_SIZE):
                i2c_buffer[i] = 0

        i2c.set_reg_value(0x01, i2c_buffer)

        img.draw_string(0, 0, "%2.1ffps" %(fps), color=(0, 60, 128), scale=2.0)
        lcd.display(img)

except Exception as e:
    raise e
finally:
    kpu.deinit()
