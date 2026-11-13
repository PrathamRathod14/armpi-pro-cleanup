#!/usr/bin/env python3
# Find the blue cube, line the gripper up over it with inverse kinematics,
# grab it and place it on the robot's left. Retries until it succeeds.
#   python3 ~/fetch_blue.py look   detection test only: nothing moves
#   python3 ~/fetch_blue.py arm    arm only: the wheels never move
#   python3 ~/fetch_blue.py        full: search and drive, then grab (floor only)
# Stop: Ctrl+C. Needs the taught LOOK and DROP poses in ~/cleanup_poses.json.
import os, sys, json, time, threading, signal
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from chassis_control.msg import SetVelocity
from armpi_pro import bus_servo_control
from kinematics import ik_transform

MODE = sys.argv[1] if len(sys.argv) > 1 else 'full'
ARM_ONLY = MODE in ('arm', 'look')

# ---------------- detection ----------------
STRICT = ((95, 110, 120), (130, 255, 255))   # to FIND the cube: clearly blue and bright
LOOSE = ((90, 70, 60), (135, 255, 255))      # to KEEP tracking it: also shaded faces and highlights
MIN_AREA = 150             # smallest patch to find the cube (pixels)
MIN_SOLIDITY = 0.75        # cube must be a solid patch (rejects ragged shapes)
MIN_FILL = 0.45            # cube must fill at least this much of its box
MAX_ASPECT = 2.5           # cube may not be long and thin
HOLD_FRAMES = 8            # keep the last position through this many missed pictures (~0.5 s)
SMOOTH = 0.5               # 1 = jump to each new position, lower = smoother
FIND_FRAMES = 3            # fresh sightings needed before the cube counts as found
# ---------------- wheels (full mode only) ----------------
TURN_RATE = 0.25
TURN_SIGN = 1              # reversed automatically if the robot turns away from the cube
DRIVE_SPEED = 45.0         # mm/s while far
CREEP_SPEED = 30.0         # mm/s when close
CENTRE_BAND = 0.15
NEAR_STOP = 0.82           # stop driving when the cube is this low in the picture
SEARCH_TIME = 60
NUDGE_TIME = 1.2
# ---------------- arm ----------------
CENTRE_X, CENTRE_Y = 320, 410   # where the cube appears when the gripper is right above it
HOVER_Z, GRAB_Z = 0.03, -0.08   # gripper height while tracking / when grabbing (metres)
MIN_REACH, MAX_REACH = 0.12, 0.25
GRIP_OPEN, GRIP_CLOSE = 120, 450   # low = open on this robot
X_GAIN, Y_GAIN = 0.03, 0.000015
DEADBAND, ON_TARGET, JUMP = 10, 20, 140
STABLE_FRAMES = 10
SLOW = 2.0                 # 1 = fast, 2 = half speed
TRACK_MS = 120
MAX_TRIES = 8
POSE_FILE = os.path.expanduser('~/cleanup_poses.json')
SCAN = [(500, 0.15), (400, 0.15), (600, 0.15), (500, 0.22), (400, 0.22), (600, 0.22)]
# -------------------------------------

ik = ik_transform.ArmIK()
lock = threading.Lock()
latest = {'seq': 0, 'strict': [], 'loose': [], 'size': (640, 480)}
status = 'starting'
last_proc = 0.0
poses = {}
joints_pub = chassis_pub = image_pub = buzzer_pub = None


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------------- vision ----------------
def blobs_in(hsv, rng, min_area, shape_check):
    mask = cv2.inRange(hsv, np.array(rng[0]), np.array(rng[1]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))   # join pieces split by highlights
    out = []
    for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
        a = cv2.contourArea(c)
        if a < min_area:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        aspect = max(bw, bh) / float(max(1, min(bw, bh)))
        if shape_check:
            hull = cv2.contourArea(cv2.convexHull(c))
            if (a / hull if hull > 0 else 0) < MIN_SOLIDITY or a / float(bw * bh) < MIN_FILL or aspect > MAX_ASPECT:
                continue
        elif aspect > 4:
            continue
        m = cv2.moments(c)
        out.append((m['m10'] / m['m00'], m['m01'] / m['m00'], a, (x, y, bw, bh)))
    out.sort(key=lambda b: -b[2])
    return out


def detect(img):
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
    return blobs_in(hsv, STRICT, MIN_AREA, True), blobs_in(hsv, LOOSE, MIN_AREA // 2, False)


def on_image(msg):
    global last_proc
    if time.time() - last_proc < 0.06:
        return
    last_proc = time.time()
    img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return
    h, w = img.shape[:2]
    strict, loose = detect(img)
    with lock:
        latest['seq'] += 1
        latest['strict'], latest['loose'], latest['size'] = strict, loose, (w, h)
    for (cx, cy, a, (x, y, bw, bh)) in loose:
        cv2.rectangle(img, (x, y), (x + bw, y + bh), (255, 160, 0), 1)
    for i, (cx, cy, a, (x, y, bw, bh)) in enumerate(strict):
        cv2.rectangle(img, (x, y), (x + bw, y + bh), (255, 0, 0), 3 if i == 0 else 1)
    if not ARM_ONLY or MODE == 'arm':
        cv2.drawMarker(img, (CENTRE_X, CENTRE_Y), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
    cv2.putText(img, status, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    out = Image()
    out.header = msg.header
    out.height, out.width, out.encoding, out.step = h, w, 'bgr8', w * 3
    out.data = img.tobytes()
    image_pub.publish(out)


def next_frame(last_seq, timeout=1.0):
    t0 = time.time()
    while not rospy.is_shutdown() and time.time() - t0 < timeout:
        with lock:
            if latest['seq'] != last_seq:
                return latest['seq'], list(latest['strict']), list(latest['loose']), latest['size']
        time.sleep(0.01)
    with lock:
        return latest['seq'], [], [], latest['size']


class Tracker(object):
    """Follows one cube from picture to picture.
    Finds it with the strict colour + shape test, then keeps it with the looser colour test
    near its last position, smooths the position, and rides through short dropouts."""

    def __init__(self):
        self.pos, self.missed, self.fresh = None, 0, 0

    def update(self, strict, loose):
        cand = None
        if self.pos is None:
            cand = strict[0] if strict else None
        else:
            near = [b for b in strict + loose
                    if (b[0] - self.pos[0]) ** 2 + (b[1] - self.pos[1]) ** 2 < JUMP ** 2]
            if near:
                cand = min(near, key=lambda b: (b[0] - self.pos[0]) ** 2 + (b[1] - self.pos[1]) ** 2)
        if cand:
            if self.pos is None:
                self.pos = (cand[0], cand[1])
            else:
                self.pos = (self.pos[0] + SMOOTH * (cand[0] - self.pos[0]),
                            self.pos[1] + SMOOTH * (cand[1] - self.pos[1]))
            self.missed = 0
            self.fresh += 1
            return self.pos, True
        if self.pos is not None:
            self.missed += 1
            if self.missed <= HOLD_FRAMES:
                return self.pos, False          # briefly not seen: keep the last position
        self.pos, self.fresh = None, 0
        return None, False


def visible(frames=8):
    """Watch a few pictures; return the cube position if it was seen reliably."""
    tr, seq = Tracker(), latest['seq']
    for _ in range(frames):
        seq, strict, loose, _ = next_frame(seq)
        pos, fresh = tr.update(strict, loose)
        if tr.fresh >= FIND_FRAMES and pos:
            return pos
    return None


# ---------------- motion ----------------
def say(text):
    global status
    status = text
    print(text)


def pause(sec):
    time.sleep(sec * SLOW)


def drive(vel, direction, angular):
    if ARM_ONLY and (vel or angular):
        return                                  # never move the wheels in arm / look mode
    chassis_pub.publish(float(vel), float(direction), float(angular))


def stop():
    drive(0, 0, 0)


def servos(ms, pairs):
    bus_servo_control.set_servos(joints_pub, int(ms * SLOW), tuple((int(i), int(p)) for i, p in pairs))


def ik_move(reach, z, base, ms, extra=()):
    t = ik.setPitchRanges((0, round(reach, 4), z), -180, -180, 0)
    if not t:
        return None
    sd = t[1]
    servos(ms, tuple(extra) + ((3, sd['servo3']), (4, sd['servo4']), (5, sd['servo5']), (6, base)))
    return t


def arm_look():
    lp = poses['look']
    servos(1500, ((1, GRIP_OPEN), (2, lp[2]), (3, lp[3]), (4, lp[4]), (5, lp[5]), (6, lp[6])))
    pause(1.6)


def arm_down():
    ik_move(0.15, HOVER_Z, 500, 1500, extra=((1, GRIP_OPEN), (2, 500)))
    pause(1.6)


def nudge(direction):
    drive(CREEP_SPEED, direction, 0)
    time.sleep(NUDGE_TIME)
    stop()
    time.sleep(0.5)


# ---------------- 1. find and drive (full mode) ----------------
def search():
    say('Searching: turning to find the blue cube')
    tr, seq, t0 = Tracker(), latest['seq'], time.time()
    while not rospy.is_shutdown() and time.time() - t0 < SEARCH_TIME:
        seq, strict, loose, _ = next_frame(seq)
        pos, fresh = tr.update(strict, loose)
        if pos:
            stop()
            if tr.fresh >= FIND_FRAMES:
                return True
        else:
            drive(0, 90, TURN_SIGN * TURN_RATE)
    stop()
    return False


def approach():
    global TURN_SIGN
    say('Driving to the cube')
    tr, seq = Tracker(), latest['seq']
    t_check, last_err, t0 = time.time(), None, time.time()
    while not rospy.is_shutdown() and time.time() - t0 < 90:
        seq, strict, loose, (w, h) = next_frame(seq)
        pos, fresh = tr.update(strict, loose)
        if pos is None:
            stop()
            say('Lost the cube while driving')
            return False
        if not fresh:
            stop()                              # briefly not seen: wait instead of guessing
            continue
        ex = (pos[0] - w / 2.0) / (w / 2.0)
        if pos[1] >= NEAR_STOP * h and abs(ex) < CENTRE_BAND:
            stop()
            return True
        if abs(ex) > CENTRE_BAND:
            drive(0, 90, -TURN_SIGN * TURN_RATE * (1 if ex > 0 else -1))
            if time.time() - t_check > 0.8:
                if last_err is not None and abs(ex) > last_err + 0.08:
                    TURN_SIGN = -TURN_SIGN
                    print('  turning the other way')
                last_err, t_check = abs(ex), time.time()
        else:
            last_err = None
            drive(DRIVE_SPEED if pos[1] < 0.6 * h else CREEP_SPEED, 90, -TURN_SIGN * TURN_RATE * 0.5 * ex)
    stop()
    return False


# ---------------- 2. line the gripper up (inverse kinematics) ----------------
def arm_find():
    arm_down()
    if visible():
        return 500, 0.15
    say('Looking around with the arm')
    for base, reach in SCAN:
        ik_move(reach, HOVER_Z, base, 800)
        pause(1.0)
        if visible():
            return base, reach
    return None


def track(base, reach):
    say('Lining the gripper up over the cube')
    x_dis, y_dis = float(base), float(reach)
    tr, seq = Tracker(), latest['seq']
    stable, far, near, t0, shown = 0, 0, 0, time.time(), 0.0
    while not rospy.is_shutdown() and time.time() - t0 < 40:
        seq, strict, loose, _ = next_frame(seq)
        pos, fresh = tr.update(strict, loose)
        if pos is None:
            return 'lost', None
        if not fresh:
            continue                            # briefly not seen: hold still, keep the count
        cx, cy = pos
        dx = X_GAIN * (CENTRE_X - cx) if abs(cx - CENTRE_X) >= DEADBAND else 0.0
        dy = Y_GAIN * (CENTRE_Y - cy) if abs(cy - CENTRE_Y) >= DEADBAND else 0.0
        x_dis = clamp(x_dis + dx, 200, 800)
        y_dis = clamp(y_dis + dy, MIN_REACH, MAX_REACH)
        t = ik_move(y_dis, HOVER_Z, int(x_dis), TRACK_MS / SLOW)
        if time.time() - shown > 0.5:
            shown = time.time()
            print('  cube in picture (%d, %d) -> arm: base %d, reach %.3f m' % (cx, cy, x_dis, y_dis))
        far = far + 1 if (y_dis >= MAX_REACH and cy < CENTRE_Y - ON_TARGET) else 0
        near = near + 1 if (y_dis <= MIN_REACH and cy > CENTRE_Y + ON_TARGET) else 0
        if far >= 15:
            return 'far', None
        if near >= 15:
            return 'near', None
        if abs(cx - CENTRE_X) < ON_TARGET and abs(cy - CENTRE_Y) < ON_TARGET and t:
            stable += 1
            if stable >= STABLE_FRAMES:
                print('  above the cube: base %d, reach %.3f m' % (x_dis, y_dis))
                return 'ok', (int(x_dis), y_dis, t[2])
        else:
            stable = 0
    return 'lost', None


def line_up():
    for adjust in range(4):
        found = arm_find()
        if not found:
            result = 'far' if adjust % 2 == 0 else 'near'
        else:
            result, data = track(*found)
            if result == 'ok':
                return data
        if result == 'far':
            if ARM_ONLY:
                say('Cube too far for the arm - move it a little closer')
                time.sleep(4)
            else:
                say('Cube a little too far for the arm - creeping forward')
                nudge(90)
        elif result == 'near':
            if ARM_ONLY:
                say('Cube too close for the arm - move it a little away')
                time.sleep(4)
            else:
                say('Cube too close for the arm - backing up a little')
                nudge(270)
        else:
            return None
    return None


# ---------------- 3. grab, place, check ----------------
def grab(base, reach, pitch):
    say('Grabbing')
    offset = -0.04 + (pitch + 180) * (0.07 / 30.0)   # Hiwonder's reach correction
    buzzer_pub.publish(0.1)
    servos(500, ((1, GRIP_OPEN),))
    pause(0.5)
    if not ik_move(reach + offset, GRAB_Z, base, 1000):
        say('Cube is out of reach')
        return False
    pause(1.5)
    servos(500, ((1, GRIP_CLOSE),))
    pause(0.8)
    servos(1500, ((1, GRIP_CLOSE), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))   # lift
    pause(1.6)
    return True


def place_left():
    say('Placing on the left')
    d = poses['drop']
    servos(1200, ((6, d[6]),))
    pause(1.3)
    servos(1500, ((3, d[3]), (4, d[4]), (5, d[5])))
    pause(1.8)
    servos(500, ((1, GRIP_OPEN),))
    pause(0.8)
    servos(1200, ((3, 80), (4, 825), (5, 625)))
    pause(1.3)


def still_there(base, reach):
    ik_move(reach, HOVER_Z, base, 1500)
    pause(1.8)
    pos = visible()
    return bool(pos and abs(pos[0] - CENTRE_X) < 80 and abs(pos[1] - CENTRE_Y) < 80)


# ---------------- detection test ----------------
def look_test():
    say('Detection test - nothing moves. Ctrl+C to stop.')
    tr, seq = Tracker(), latest['seq']
    seen = held = total = 0
    t_print = time.time()
    while not rospy.is_shutdown():
        seq, strict, loose, _ = next_frame(seq)
        pos, fresh = tr.update(strict, loose)
        total += 1
        seen += 1 if fresh else 0
        held += 1 if (pos and not fresh) else 0
        if time.time() - t_print > 1.0:
            where = 'at (%d, %d)' % pos if pos else 'not tracked'
            print('  seen in %d of %d pictures, held %d, cube %s, strict %d / loose %d patches'
                  % (seen, total, held, where, len(strict), len(loose)))
            seen = held = total = 0
            t_print = time.time()


def main():
    global joints_pub, chassis_pub, image_pub, buzzer_pub
    rospy.init_node('fetch_blue')
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    chassis_pub = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    image_pub = rospy.Publisher('/cleanup/image', Image, queue_size=1)
    buzzer_pub = rospy.Publisher('/sensor/buzzer', Float32, queue_size=1)
    rospy.on_shutdown(stop)
    signal.signal(signal.SIGHUP, lambda *a: rospy.signal_shutdown('hangup'))
    try:
        with open(POSE_FILE) as f:
            for k, v in json.load(f).items():
                if isinstance(v, dict):
                    poses[k] = dict((int(i), int(p)) for i, p in v.items())
    except (IOError, OSError, ValueError):
        pass
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, on_image, queue_size=1, buff_size=2 ** 24)
    rospy.sleep(1.0)
    if latest['seq'] == 0:
        print('No camera pictures - is the camera running?')
        return
    print('Camera view: http://ubuntu.local:8080/stream_viewer?topic=/cleanup/image')
    if MODE == 'look':
        look_test()
        return
    if 'look' not in poses or 'drop' not in poses:
        print('Teach the LOOK (key 1) and DROP (key 4) poses first: python3 ~/cleanup.py teach')
        return
    if ARM_ONLY:
        print('Arm only: the wheels will not move. Put the cube 12-25 cm in front. Ctrl+C stops.')
    else:
        print('This mode DRIVES. Use it on the floor. Press Ctrl+C to stop.')
    for attempt in range(1, MAX_TRIES + 1):
        if rospy.is_shutdown():
            return
        print('--- Attempt %d of %d ---' % (attempt, MAX_TRIES))
        if not ARM_ONLY:
            arm_look()
            if not visible() and not search():
                say('No blue cube found anywhere around the robot')
                return
            if not approach():
                continue
        above = line_up()
        if not above:
            continue
        base, reach, pitch = above
        if not grab(base, reach, pitch):
            continue
        place_left()
        if still_there(base, reach):
            say('The cube is still there - the grab missed, trying again')
            continue
        arm_look()
        say('Done: blue cube placed on the left')
        return
    stop()
    arm_look()
    say('Gave up after %d attempts' % MAX_TRIES)


if __name__ == '__main__':
    main()
