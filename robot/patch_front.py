import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'class FrontCam(' in s:
    print('already patched')
    sys.exit(0)

FRONT = r'''# ---------------- front camera: ACEBOTT K210 module on the chassis ----------------
FRONT_PORT = '/dev/ttyUSB0'
FRONT_W, FRONT_H = 320, 240
FRONT_BLUE = (20, 85, -25, 15, -70, -15)   # LAB colour range of the blue cube: L min/max, A min/max, B min/max
FRONT_MIN_PIXELS = 60      # smallest blue patch (the cube far away is small)
FRONT_VFLIP = 0            # 1 if the module's picture is upside down
FRONT_HMIRROR = 0          # 1 if the module's picture is mirrored
FRONT_STOP_Y = 200         # stop driving when the cube's centre is this low in the front picture (0-240)
FRONT_UNDER = 0.8          # cube vanished after being this low = it went under the camera = close enough

K210_SCRIPT = """
import sensor, time
try:
    import lcd
    lcd.init()
    has_lcd = True
except Exception:
    has_lcd = False
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip({vflip})
sensor.set_hmirror({hmirror})
sensor.skip_frames(time=800)
th = [{th}]
while True:
    img = sensor.snapshot()
    bl = img.find_blobs(th, pixels_threshold={px}, area_threshold={px}, merge=True)
    if bl:
        b = max(bl, key=lambda q: q.pixels())
        img.draw_rectangle(b.rect(), color=(0, 255, 0))
        print('B,%d,%d,%d,%d,%d' % (b.cx(), b.cy(), b.w(), b.h(), b.pixels()))
    else:
        print('N')
    if has_lcd:
        lcd.display(img)
"""


def parse_front(line):
    """'B,cx,cy,w,h,pixels' -> tuple, 'N' -> None, anything else -> False (not a detection line)."""
    if 'B,' in line:
        try:
            v = [int(x) for x in line[line.index('B,') + 2:].split(',')[:5]]
            return tuple(v) if len(v) == 5 else False
        except ValueError:
            return False
    if line.endswith('N'):
        return None
    return False


class FrontCam(threading.Thread):
    """Runs a small detection program on the K210 module (sent over USB, not saved on it)
    and reads back where it sees the blue cube."""

    def __init__(self, port):
        threading.Thread.__init__(self)
        self.daemon = True
        self.port, self.ser, self.lock = port, None, threading.Lock()
        self.det, self.t, self.count, self.running = None, 0.0, 0, True

    def send_script(self):
        import serial
        self.ser = s = serial.Serial(self.port, 115200, timeout=0.1)
        time.sleep(2.5)                     # opening the port can restart the module: let it boot
        s.reset_input_buffer()
        s.write(b'\r\x03\x03')              # stop whatever program it is running
        time.sleep(0.5)
        s.write(b'\r\x01')                  # raw REPL
        time.sleep(0.5)
        resp = s.read(4000)
        code = K210_SCRIPT.format(vflip=FRONT_VFLIP, hmirror=FRONT_HMIRROR,
                                  th=str(tuple(FRONT_BLUE)), px=FRONT_MIN_PIXELS).encode()
        if b'raw REPL' not in resp:
            s.write(b'\x05')                # fall back to paste mode
            time.sleep(0.3)
        for i in range(0, len(code), 128):
            s.write(code[i:i + 128])
            time.sleep(0.02)
        s.write(b'\x04')                    # run it

    def run(self):
        try:
            self.send_script()
        except Exception as e:
            print('Front camera: cannot use %s (%s)' % (self.port, e))
            return
        buf = b''
        while self.running:
            try:
                data = self.ser.read(512)
            except Exception:
                break
            if not data:
                continue
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                d = parse_front(line.strip().decode('utf-8', 'replace'))
                if d is False:
                    continue
                with self.lock:
                    self.det, self.t = d, time.time()
                    self.count += 1

    def latest(self, last_count, timeout=1.0):
        end = time.time() + timeout
        while time.time() < end and not rospy.is_shutdown():
            with self.lock:
                if self.count != last_count:
                    return self.count, self.det
            time.sleep(0.01)
        return last_count, None

    def stop(self):
        self.running = False
        try:
            self.ser.write(b'\x03\x03\x02')  # stop the program, leave raw REPL
            self.ser.close()
        except Exception:
            pass


front = None


def arm_carry():
    """Arm raised and folded, out of the front camera's way while driving."""
    servos(1500, ((1, GRIP_OPEN), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))
    pause(1.6)


def front_search():
    say('Searching with the front camera')
    seq, seen, t0 = front.count, 0, time.time()
    while not rospy.is_shutdown() and time.time() - t0 < SEARCH_TIME:
        seq, det = front.latest(seq)
        if det:
            stop()
            seen += 1
            if seen >= 3:
                return True
        else:
            seen = 0
            drive(0, 90, TURN_SIGN * TURN_RATE)
    stop()
    return False


def front_approach():
    """Steer the wheels by the front camera until the cube is close enough for the arm."""
    global TURN_SIGN
    say('Driving to the cube (front camera)')
    seq, low, last_err, last_turn, flipped = front.count, False, None, None, False
    t_lost = t_check = t0 = time.time()
    shown = 0.0
    while not rospy.is_shutdown() and time.time() - t0 < 90:
        seq, det = front.latest(seq)
        if not det:
            if low:
                stop()
                print('  cube went under the front camera - close enough')
                return True
            if last_turn is not None and time.time() - t_lost < 3.0:
                if not flipped:                 # lost it while turning: we turned the wrong way
                    TURN_SIGN = -TURN_SIGN
                    flipped = True
                    print('  lost the cube while turning - turning back the other way')
                drive(0, 90, -TURN_SIGN * TURN_RATE * last_turn)
                continue
            stop()
            if time.time() - t_lost > 1.5:
                say('Lost the cube while driving')
                return False
            continue
        t_lost, flipped = time.time(), False
        cx, cy = det[0], det[1]
        low = cy > FRONT_UNDER * FRONT_H
        ex = (cx - FRONT_W / 2.0) / (FRONT_W / 2.0)
        if time.time() - shown > 0.5:
            shown = time.time()
            print('  front camera: cube at (%d, %d), size %d' % (cx, cy, det[4]))
        if cy >= FRONT_STOP_Y and abs(ex) < CENTRE_BAND:
            stop()
            return True
        if abs(ex) > CENTRE_BAND:
            last_turn = 1 if ex > 0 else -1
            drive(0, 90, -TURN_SIGN * TURN_RATE * last_turn)
            if time.time() - t_check > 0.4:
                if last_err is not None and abs(ex) > last_err + 0.05:
                    TURN_SIGN = -TURN_SIGN
                    print('  turning the other way')
                last_err, t_check = abs(ex), time.time()
        else:
            last_err, last_turn = None, None
            drive(DRIVE_SPEED if cy < 0.6 * FRONT_H else CREEP_SPEED, 90, -TURN_SIGN * TURN_RATE * 0.5 * ex)
    stop()
    return False


def front_look():
    say('Front camera test - nothing moves. Ctrl+C to stop.')
    seq, n, seen, t_print = front.count, 0, 0, time.time()
    while not rospy.is_shutdown():
        seq, det = front.latest(seq)
        n += 1
        seen += 1 if det else 0
        if time.time() - t_print > 1.0:
            where = 'at (%d, %d), %dx%d, %d pixels' % det if det else 'not seen'
            print('  front camera: %d pictures, cube seen in %d | now %s' % (n, seen, where))
            n, seen, t_print = 0, 0, time.time()


'''

R = [
    ("ARM_ONLY = MODE in ('arm', 'look', 'vlm', 'calibgrab')\n",
     "ARM_ONLY = MODE in ('arm', 'look', 'vlm', 'calibgrab', 'frontlook')\n"),
    ("def main():\n    global joints_pub, chassis_pub, image_pub, buzzer_pub, CENTRE_X, CENTRE_Y\n",
     FRONT + "def main():\n    global joints_pub, chassis_pub, image_pub, buzzer_pub, CENTRE_X, CENTRE_Y, front\n"),
    ("    rospy.sleep(1.0)\n    if latest['seq'] == 0:\n",
     "    if MODE in ('front', 'frontlook'):\n"
     "        front = FrontCam(FRONT_PORT)\n"
     "        front.start()\n"
     "        rospy.on_shutdown(front.stop)\n"
     "        print('Starting the front camera program on the K210 module...')\n"
     "        t_wait = time.time()\n"
     "        while front.count == 0 and time.time() - t_wait < 12 and not rospy.is_shutdown():\n"
     "            time.sleep(0.2)\n"
     "        if front.count == 0:\n"
     "            print('No results from the front camera on %s - is the module plugged in with data?' % FRONT_PORT)\n"
     "            return\n"
     "        print('Front camera running.')\n"
     "    rospy.sleep(1.0)\n    if latest['seq'] == 0:\n"),
    ("    if MODE == 'calibgrab':\n        calib_grab()\n        return\n",
     "    if MODE == 'calibgrab':\n        calib_grab()\n        return\n"
     "    if MODE == 'frontlook':\n        front_look()\n        return\n"),
    ("        if not ARM_ONLY:\n            arm_look()\n            if not visible() and not search():",
     "        if MODE == 'front':\n"
     "            arm_carry()\n"
     "            if not front_search():\n"
     "                say('The front camera sees no blue cube')\n"
     "                return\n"
     "            if not front_approach():\n"
     "                continue\n"
     "        elif not ARM_ONLY:\n            arm_look()\n            if not visible() and not search():"),
]
missing = [old[:60] for old, new in R if old not in s]
if missing:
    print('nothing changed, could not find:', missing)
    sys.exit(0)
for old, new in R:
    s = s.replace(old, new, 1)
open(path, 'w').write(s)
print('patched')
