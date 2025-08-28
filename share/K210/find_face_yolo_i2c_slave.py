import sensor, image, time, lcd
from maix import KPU
import hiwonder

i2c = hiwonder.hw_slavei2c()
send_buffer = bytearray([0] * 9)

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

            send_x = l[0]
            send_y = l[1]
            send_w = l[2]
            send_h = l[3]

            send_buffer[0] = K210_FIND_FACE_YOLO
            send_buffer[1] = send_x & 0xFF
            send_buffer[2] = (send_x >> 8) & 0xFF
            send_buffer[3] = send_y & 0xFF
            send_buffer[4] = (send_y >> 8) & 0xFF
            send_buffer[5] = send_w & 0xFF
            send_buffer[6] = (send_w >> 8) & 0xFF
            send_buffer[7] = send_h & 0xFF
            send_buffer[8] = (send_h >> 8) & 0xFF

        if not face_detected:
            for i in range(len(send_buffer)):
                send_buffer[i] = 0

        i2c.set_reg_value(0x01, send_buffer)

        img.draw_string(0, 0, "%2.1ffps" %(fps), color=(0, 60, 128), scale=2.0)
        lcd.display(img)

except Exception as e:
    raise e
finally:
    kpu.deinit()
