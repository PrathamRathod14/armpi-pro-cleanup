#!/usr/bin/env python3
# The ACEBOTT K210 module (front camera) as a ROS camera, plus blue-cube detection on the module.
#   python3 ~/front_cam.py          picture + detection
#   python3 ~/front_cam.py stats    also print the colour inside the red centre square (for tuning)
# Publishes:
#   /front_cam/image   sensor_msgs/Image (bgr8): see http://ROBOT_IP:8080/stream_viewer?topic=/front_cam/image
#   /front_cam/target  std_msgs/String, JSON {"found", "cx", "cy", "w", "h", "pixels", "img_w", "img_h"}
# Stop with Ctrl+C. Switching the module off and on brings back ACEBOTT's own program.
import sys, time, json
import numpy as np
import cv2
import rospy
import serial
from sensor_msgs.msg import Image
from std_msgs.msg import String

PORT = '/dev/ttyUSB0'
FAST_BAUD = 921600           # module sends pictures faster at this speed; 0 = stay at 115200
BLUE = (20, 85, -25, 15, -70, -15)   # LAB range of the blue cube: L min/max, A min/max, B min/max
MIN_PIXELS = 60              # smallest blue patch counted as the cube
VFLIP, HMIRROR = 0, 0        # set to 1 if the picture is upside down / mirrored
JPEG_QUALITY = 35            # lower = smaller pictures = faster
STATS = len(sys.argv) > 1 and sys.argv[1] == 'stats'

SCRIPT = """
import sensor, time
from machine import UART
try:
    import lcd
    lcd.init()
    has_lcd = True
except Exception:
    has_lcd = False
repl = UART.repl_uart()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip({vflip})
sensor.set_hmirror({hmirror})
sensor.skip_frames(time=1000)
th = [{th}]
roi = (140, 100, 40, 40)
if {baud}:
    time.sleep_ms(300)
    repl.init({baud}, 8, None, 1, read_buf_len=4096)
    time.sleep_ms(300)
while True:
    img = sensor.snapshot()
    bl = img.find_blobs(th, pixels_threshold={px}, area_threshold={px}, merge=True)
    if bl:
        b = max(bl, key=lambda q: q.pixels())
        img.draw_rectangle(b.rect(), color=(0, 255, 0))
        line = 'B,%d,%d,%d,%d,%d\\n' % (b.cx(), b.cy(), b.w(), b.h(), b.pixels())
    else:
        line = 'N\\n'
    if {stats}:
        st = img.get_statistics(roi=roi)
        line += 'S,%d,%d,%d\\n' % (st.l_mean(), st.a_mean(), st.b_mean())
        img.draw_rectangle(roi, color=(255, 0, 0))
    if has_lcd:
        lcd.display(img)
    c = img.compress(quality={q})
    data = c.to_bytes() if hasattr(c, 'to_bytes') else bytearray(c)
    repl.write(line)
    repl.write(data)
"""


def open_port():
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = PORT, 115200, 0.05
    s.dtr = False                    # do not hold the module's reset/boot pins
    s.rts = False
    s.open()
    return s


def reset_module(s):
    s.baudrate = 115200
    s.rts = True
    time.sleep(0.2)
    s.rts = False


def start(s, baud):
    print('Starting the module (about 10 s)...')
    time.sleep(6)
    s.reset_input_buffer()
    got, end = b'', time.time() + 8
    while b'>>>' not in got and time.time() < end:      # stop ACEBOTT's program
        s.write(b'\x03')
        time.sleep(0.3)
        got += s.read(2000)
    if b'>>>' not in got:
        raise IOError('the module did not stop its own program (no >>> prompt)')
    time.sleep(0.5)
    s.read(4000)
    s.write(b'\r\x01')                                   # raw REPL
    time.sleep(0.5)
    s.read(4000)
    code = SCRIPT.format(vflip=VFLIP, hmirror=HMIRROR, th=str(tuple(BLUE)), px=MIN_PIXELS,
                         baud=baud, stats=1 if STATS else 0, q=JPEG_QUALITY).encode()
    for i in range(0, len(code), 128):
        s.write(code[i:i + 128])
        time.sleep(0.02)
    s.write(b'\x04')                                     # run it
    if baud:
        time.sleep(1.3)
        s.baudrate = baud
    s.reset_input_buffer()


def pump(buf):
    """Split the byte stream into JPEG pictures and text lines."""
    frames, lines = [], []
    while True:
        j = buf.find(b'\xff\xd8')
        head = buf if j < 0 else buf[:j]
        k = head.rfind(b'\n')
        if k >= 0:
            lines += [ln.strip().decode('ascii', 'ignore') for ln in bytes(head[:k]).split(b'\n')]
        if j < 0:
            if k >= 0:
                del buf[:k + 1]
            break
        e = buf.find(b'\xff\xd9', j + 2)
        if e < 0:
            del buf[:j]
            break
        frames.append(bytes(buf[j:e + 2]))
        del buf[:e + 2]
    if len(buf) > 300000:
        del buf[:]
    return frames, lines


def parse_line(t):
    if 'B,' in t:
        try:
            v = [int(x) for x in t[t.index('B,') + 2:].split(',')[:5]]
            if len(v) == 5:
                return 'B', v
        except ValueError:
            pass
    elif 'S,' in t:
        try:
            v = [int(x) for x in t[t.index('S,') + 2:].split(',')[:3]]
            if len(v) == 3:
                return 'S', v
        except ValueError:
            pass
    elif t.endswith('N'):
        return 'N', None
    return None, None


def run(s, img_pub, tgt_pub, probe=0.0):
    """Read the module; returns False if nothing arrived within `probe` seconds (when probe > 0)."""
    buf = bytearray()
    t0 = t_print = time.time()
    n_img = n_det = 0
    stats = None
    while not rospy.is_shutdown():
        buf += s.read(8192)
        frames, lines = pump(buf)
        for t in lines:
            kind, v = parse_line(t)
            if kind == 'B':
                n_det += 1
                tgt_pub.publish(json.dumps({'found': True, 'cx': v[0], 'cy': v[1], 'w': v[2], 'h': v[3],
                                            'pixels': v[4], 'img_w': 320, 'img_h': 240}))
            elif kind == 'N':
                n_det += 1
                tgt_pub.publish(json.dumps({'found': False, 'img_w': 320, 'img_h': 240}))
            elif kind == 'S':
                stats = v
        for f in frames:
            img = cv2.imdecode(np.frombuffer(f, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            n_img += 1
            m = Image()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = 'front_cam'
            m.height, m.width = img.shape[:2]
            m.encoding, m.step = 'bgr8', img.shape[1] * 3
            m.data = img.tobytes()
            img_pub.publish(m)
        if probe and time.time() - t0 > probe:
            if n_img == 0 and n_det == 0:
                return False
            probe = 0.0
        if time.time() - t_print > 3:
            dt = time.time() - t_print
            msg = '  %.1f pictures/s, %.1f detections/s' % (n_img / dt, n_det / dt)
            if STATS and stats:
                msg += ' | red square colour: L %d  A %d  B %d' % tuple(stats)
            print(msg)
            n_img = n_det = 0
            t_print = time.time()
    return True


def main():
    rospy.init_node('front_cam')
    img_pub = rospy.Publisher('/front_cam/image', Image, queue_size=1)
    tgt_pub = rospy.Publisher('/front_cam/target', String, queue_size=1)
    s = open_port()
    rospy.on_shutdown(lambda: (s.write(b'\x03\x03\x02'), s.close()) if s.is_open else None)
    try:
        start(s, FAST_BAUD)
        if FAST_BAUD:
            print('Front camera running at %d baud. Ctrl+C stops.' % FAST_BAUD)
            if run(s, img_pub, tgt_pub, probe=6.0):
                return
            print('No data at the fast speed - restarting the module at the normal speed (slower pictures)')
            reset_module(s)
        start(s, 0)
        print('Front camera running at 115200 baud. Ctrl+C stops.')
        run(s, img_pub, tgt_pub)
    except IOError as e:
        print('Front camera: %s' % e)


if __name__ == '__main__':
    main()
