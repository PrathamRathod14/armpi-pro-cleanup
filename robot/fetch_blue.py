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
from std_msgs.msg import Float32, String
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from chassis_control.msg import SetVelocity
from armpi_pro import bus_servo_control
from kinematics import ik_transform

MODE = sys.argv[1] if len(sys.argv) > 1 else 'full'
ARM_ONLY = MODE in ('arm', 'look', 'vlm', 'calibgrab', 'frontlook', 'frontarm')

# ---------------- detection ----------------
STRICT = ((95, 110, 120), (130, 255, 255))   # to FIND the cube: clearly blue and bright
LOOSE = ((90, 70, 60), (135, 255, 255))      # to KEEP tracking it: also shaded faces and highlights
PICK_RED = True            # also pick red cubes
STRICT_RED = (((0, 120, 80), (8, 255, 255)), ((170, 120, 80), (180, 255, 255)))
LOOSE_RED = (((0, 90, 60), (9, 255, 255)), ((165, 90, 60), (180, 255, 255)))
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
WRIST_ALIGN = True         # turn the gripper to match the cube's sides before grabbing
WRIST_SIGN = 1             # reversed automatically if turning the wrist makes it worse
WRIST_UNITS_PER_DEG = 1000 / 240.0
GRIP_EXTRA = 25            # second, firmer squeeze after the first close
LIFT_FIRST = 0.04          # lift straight up this far (metres) before swinging away
FINE_TARGET, FINE_DEADBAND = 8, 4   # exact centring before grabbing (pixels)
SETTLE_OPEN = 300          # loosen to this after the first close so the cube settles in the middle
GRAB_TRIES = 3             # grabs on the spot before starting the whole attempt again
HELD_GROWTH = 1.8          # cube in the gripper looks at least this much bigger than on the floor
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
def blobs_in(hsv, rngs, min_area, shape_check):
    mask = None
    for rng in rngs:                                  # one or more colour ranges, joined
        m = cv2.inRange(hsv, np.array(rng[0]), np.array(rng[1]))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
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
        ang = cv2.minAreaRect(c)[2] % 90.0
        ang = ang - 90.0 if ang > 45.0 else ang          # -45..45 degrees: how far the sides are turned
        out.append((m['m10'] / m['m00'], m['m01'] / m['m00'], a, (x, y, bw, bh), ang))
    out.sort(key=lambda b: -b[2])
    return out


def detect(img):
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
    strict = (STRICT,) + (STRICT_RED if PICK_RED else ())
    loose = (LOOSE,) + (LOOSE_RED if PICK_RED else ())
    return blobs_in(hsv, strict, MIN_AREA, True), blobs_in(hsv, loose, MIN_AREA // 2, False)


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
    for (cx, cy, a, (x, y, bw, bh)) in [q[:4] for q in loose]:
        cv2.rectangle(img, (x, y), (x + bw, y + bh), (255, 160, 0), 1)
    for i, (cx, cy, a, (x, y, bw, bh)) in enumerate([q[:4] for q in strict]):
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
        self.pos, self.missed, self.fresh, self.angle, self.area = None, 0, 0, 0.0, 0

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
            self.angle = cand[4]
            self.area = cand[2]
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
VOICE = 'auto'             # 'auto' = talk if a speaker and espeak are there, else beep; 'beep'; 'off'
VOICE_SPEED = 150          # words per minute
OK, BAD, INFO = (0.08, 0.08), (0.45,), (0.04,)     # buzzer patterns (seconds of each beep)
SAYINGS = [                # (start of a status message, what the robot says, beeps without a speaker)
    ('Searching',               'Searching for cubes.', INFO),
    ('Grabbing',                "I'm trying to grab it.", INFO),
    ('Missed the cube',         'Oops, I missed. Trying again.', BAD),
    ('Could not get hold',      'I could not get hold of it.', BAD),
    ('Lost the cube',           'I lost the cube.', BAD),
    ('The cube is still there', 'Hmm, the cube is still there. Trying again.', BAD),
    ('Placing in the basket',   'Putting it in my basket.', INFO),
    ('Done',                    'Done! The cube is in my basket.', OK),
    ('Cube too far',            'That cube is too far for my arm. Please move it closer.', BAD),
    ('Cube too close',          'That cube is too close. Please move it back a little.', BAD),
    ('Cube a little too far',   'Moving a little closer.', INFO),
    ('Finished',                'All done! Every cube is in my basket.', OK),
    ('Gave up',                 'Sorry, I could not do it.', BAD),
    ('The front camera does not see', "I can't see any cube.", BAD),
    ('The front camera sees no', "I can't see any cube.", BAD),
    ('No blue cube found',      "I can't find any cube.", BAD),
    ('No cube found',           "I can't find any cube.", BAD),
]
_voice_q, _voice_cmd, _voice_last = None, None, {}


def _voice_worker():
    import subprocess
    while True:
        text, beeps = _voice_q.get()
        try:
            if _voice_cmd:
                subprocess.call(_voice_cmd + [text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            elif buzzer_pub is not None:
                for b in beeps:
                    buzzer_pub.publish(b)
                    time.sleep(b + 0.12)
        except Exception as e:
            print('  (voice: %s)' % e)


def talk(text, beeps=INFO):
    """Say something in the background (or beep). The same sentence is not repeated within 15 s."""
    global _voice_q, _voice_cmd
    if VOICE == 'off' or time.time() - _voice_last.get(text, 0) < 15:
        return
    _voice_last[text] = time.time()
    if _voice_q is None:
        import queue, shutil
        _voice_q = queue.Queue()
        if VOICE == 'auto':
            for prog in ('espeak-ng', 'espeak', 'spd-say'):
                if shutil.which(prog):
                    _voice_cmd = [prog, '-s', str(VOICE_SPEED)] if prog != 'spd-say' else [prog, '-w']
                    break
        print('  (voice: %s)' % ('talking with ' + _voice_cmd[0] if _voice_cmd else 'buzzer beeps'))
        threading.Thread(target=_voice_worker, daemon=True).start()
    while _voice_q.qsize() >= 2:               # never fall behind: drop the oldest waiting sentence
        try:
            _voice_q.get_nowait()
        except Exception:
            break
    print('  robot says: "%s"' % text)
    _voice_q.put((text, beeps))


def say(text):
    global status
    status = text
    print(text)
    for start, words, beeps in SAYINGS:
        if text.startswith(start):
            talk(words, beeps)
            break
    if text.startswith('Cube ') and 'in the basket' in text:
        talk('Done! Looking for the next one.', OK)


def pause(sec):
    time.sleep(sec * SLOW)


MIN_BATTERY = 7200          # millivolts: below this the robot will not drive (power dips restart the Pi)
RAMP_VEL, RAMP_ANG = 12.0, 0.06   # most the wheel speed / turn rate may rise per command (gentle starts)
last_drive = [0.0, 0.0]


def battery_mv():
    try:
        import rostopic
        cls = rostopic.get_topic_class('/sensor/battery')[0]
        return int(rospy.wait_for_message('/sensor/battery', cls, timeout=3.0).data)
    except Exception:
        return None


def drive(vel, direction, angular):
    if ARM_ONLY and (vel or angular):
        return                                  # never move the wheels in arm / look mode
    if vel == 0 and angular == 0:
        last_drive[0] = last_drive[1] = 0.0
    else:                                       # ramp up instead of jumping: smaller current spikes
        vel = min(vel, last_drive[0] + RAMP_VEL)
        if abs(angular) > last_drive[1] + RAMP_ANG:
            angular = (last_drive[1] + RAMP_ANG) * (1 if angular > 0 else -1)
        last_drive[0], last_drive[1] = vel, abs(angular)
    chassis_pub.publish(float(vel), float(direction), float(angular))


def stop():
    drive(0, 0, 0)


def servos(ms, pairs):
    move_done[0] = time.time() + ms * SLOW / 1000.0
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
    if MODE == 'vlm':
        return vlm_find()
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


def track(base, reach, fine=False):
    if MODE == 'vlm':
        return vlm_track(base, reach)
    if not fine:
        say('Lining the gripper up over the cube')
    tol, db, limit = (FINE_TARGET, FINE_DEADBAND, 12) if fine else (ON_TARGET, DEADBAND, 40)
    x_dis, y_dis = float(base), float(reach)
    tr, seq = Tracker(), latest['seq']
    stable, far, near, t0, shown = 0, 0, 0, time.time(), 0.0
    while not rospy.is_shutdown() and time.time() - t0 < limit:
        seq, strict, loose, _ = next_frame(seq)
        pos, fresh = tr.update(strict, loose)
        if pos is None:
            return 'lost', None
        if not fresh:
            continue                            # briefly not seen: hold still, keep the count
        cx, cy = pos
        dx = X_GAIN * (CENTRE_X - cx) if abs(cx - CENTRE_X) >= db else 0.0
        dy = Y_GAIN * (CENTRE_Y - cy) if abs(cy - CENTRE_Y) >= db else 0.0
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
        if abs(cx - CENTRE_X) < tol and abs(cy - CENTRE_Y) < tol and t:
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


def measure_angle(frames=8):
    tr, seq, angles = Tracker(), latest['seq'], []
    for _ in range(frames * 2):
        seq, strict, loose, _ = next_frame(seq)
        pos, fresh = tr.update(strict, loose)
        if fresh:
            angles.append(tr.angle)
        if len(angles) >= frames:
            break
    return sorted(angles)[len(angles) // 2] if angles else None


def align_wrist():
    """Turn the gripper so its fingers close on two flat sides of the cube, not on its corners."""
    global WRIST_SIGN
    ang = measure_angle()
    if ang is None or abs(ang) < 4:
        return 500
    say('Turning the gripper %.0f degrees to match the cube' % ang)
    wrist = int(clamp(500 + WRIST_SIGN * ang * WRIST_UNITS_PER_DEG, 300, 700))
    servos(500, ((2, wrist),))
    pause(0.8)
    after = measure_angle()
    if after is not None and abs(after) > abs(ang):            # turned the wrong way
        WRIST_SIGN = -WRIST_SIGN
        wrist = int(clamp(500 + WRIST_SIGN * ang * WRIST_UNITS_PER_DEG, 300, 700))
        servos(500, ((2, wrist),))
        pause(0.8)
        after = measure_angle()
    print('  cube sides now at %s degrees (wrist %d)' % ('?' if after is None else '%.0f' % after, wrist))
    return wrist


# ---------------- 3. grab, place, check ----------------
def grab(base, reach, pitch, wrist=500):
    say('Grabbing')
    offset = -0.04 + (pitch + 180) * (0.07 / 30.0)   # Hiwonder's reach correction
    buzzer_pub.publish(0.1)
    servos(500, ((1, GRIP_OPEN),))
    pause(0.5)
    if not ik_move(reach + offset, GRAB_Z, base, 1000, extra=((2, wrist),)):
        say('Cube is out of reach')
        return False
    pause(1.5)
    servos(700, ((1, GRIP_CLOSE),))                      # first close: the fingers push the cube to the middle
    pause(0.9)
    servos(300, ((1, SETTLE_OPEN),))                     # loosen a little so the cube settles straight
    pause(0.5)
    servos(500, ((1, GRIP_CLOSE),))
    pause(0.8)
    servos(300, ((1, GRIP_CLOSE + GRIP_EXTRA),))         # second squeeze to seat the cube
    pause(0.5)
    ik_move(reach + offset, GRAB_Z + LIFT_FIRST, base, 800, extra=((2, wrist),))   # lift straight up first
    pause(1.0)
    return True


def carry():
    talk('Got it! I picked up the cube.', OK)
    servos(1500, ((1, GRIP_CLOSE + GRIP_EXTRA), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))
    pause(1.6)


def cube_seen(frames=12):
    """Where the cube is in the next few pictures and how big it looks (median), or None."""
    tr, seq, seen = Tracker(), latest['seq'], []
    for _ in range(frames):
        seq, strict, loose, _ = next_frame(seq)
        pos, fresh = tr.update(strict, loose)
        if fresh and pos:
            seen.append((pos[0], pos[1], tr.area))
    if len(seen) < 3:
        return None
    med = lambda i: sorted(q[i] for q in seen)[len(seen) // 2]
    return med(0), med(1), med(2)


def holding(base, reach, ref):
    """Back at the height the arm lined up at, look down: is the cube still on the floor?"""
    say('Checking the cube is in the gripper')
    ik_move(reach, HOVER_Z, base, 900)
    pause(1.2)
    now = cube_seen()
    if now is None:
        print('  no cube on the floor any more: it is in the gripper')
        return True
    cx, cy, area = now
    if ref is None:
        on_floor = abs(cx - CENTRE_X) < 90 and abs(cy - CENTRE_Y) < 90
    else:
        print('  cube size now %d pixels, before the grab %d' % (area, ref[2]))
        on_floor = area < ref[2] * HELD_GROWTH          # same size as before: still lying on the floor
    if on_floor:
        print('  the cube is still on the floor at (%d, %d)' % (cx, cy))
        return False
    print('  cube is close to the camera: it is in the gripper')
    return True


def arrange(base, reach, pitch):
    """Final careful centring right above the cube, so the fingers come down evenly on both sides."""
    say('Arranging: centring the gripper exactly over the cube')
    res, data = track(base, reach, fine=True)
    if res == 'ok':
        return data
    print('  fine centring did not settle - using the first line-up')
    return base, reach, pitch


def pick_up(base, reach, pitch, wrist):
    """Grab, check, and grab again on the spot until the cube is held. Returns True when held."""
    ref = cube_seen()
    for g in range(1, GRAB_TRIES + 1):
        if g > 1:
            say('Missed the cube - opening and grabbing again (%d of %d)' % (g, GRAB_TRIES))
            servos(500, ((1, GRIP_OPEN),))
            pause(0.6)
            res, data = track(base, reach)
            if res != 'ok':
                say('Lost the cube after the missed grab')
                return False
            base, reach, pitch = data
            if WRIST_ALIGN:
                wrist = align_wrist()
                if wrist != 500:
                    res, data = track(base, reach)
                    if res == 'ok':
                        base, reach, pitch = data
        base, reach, pitch = arrange(base, reach, pitch)
        ref = cube_seen()
        if not grab(base, reach, pitch, wrist):
            return False
        if holding(base, reach, ref):
            carry()
            return True
    say('Could not get hold of the cube after %d grabs' % GRAB_TRIES)
    servos(500, ((1, GRIP_OPEN),))
    pause(0.6)
    return False


def place_left():
    say('Placing in the basket')
    d = poses['drop']                                   # taught DROP pose: cube held above the basket
    servos(1500, ((6, d[6]),))                          # turn first, arm still raised in the carry pose
    pause(1.6)
    servos(2200, ((2, d.get(2, 500)), (3, d[3]), (4, d[4]), (5, d[5])))   # slowly over the basket
    pause(2.4)
    servos(500, ((1, GRIP_OPEN),))                      # let go: the cube drops in
    pause(0.8)
    servos(1800, ((3, 80), (4, 825), (5, 625)))         # back up to the carry pose before turning
    pause(2.0)
    servos(1500, ((2, 500), (6, 500)))
    pause(1.6)


def still_there(base, reach):
    if MODE == 'vlm':
        return still_there_vlm(base, reach)
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


# ---------------- laptop vision (YOLO-World via /vlm/target) ----------------
VLM_TOPIC = '/vlm/target'
VLM_SETTLE = 0.15          # seconds after an arm move before a picture counts as fresh
VLM_X_GAIN = 0.35          # base servo units per pixel of error, per step
VLM_Y_GAIN = 0.00025       # metres of reach per pixel of error, per step
VLM_STEPS = 30             # most line-up steps before giving up
vlm = {'msg': None, 'recv': 0.0, 'count': 0}
move_done = [0.0]          # robot time when the last arm move finishes


def on_vlm(m):
    try:
        d = json.loads(m.data)
    except ValueError:
        return
    with lock:
        vlm['msg'], vlm['recv'] = d, time.time()
        vlm['count'] += 1


def vlm_fresh(timeout=4.0):
    """Wait for a laptop result whose picture was taken after the arm stopped moving."""
    end = time.time() + timeout
    with lock:
        seen = vlm['count']
    while not rospy.is_shutdown() and time.time() < end:
        with lock:
            d, recv, c = vlm['msg'], vlm['recv'], vlm['count']
        if d is not None and c != seen:
            seen = c
            pic_time = recv - (d.get('sent_time', 0) - d.get('frame_time', 0)) - 0.1
            if pic_time > move_done[0] + VLM_SETTLE:
                return d
        time.sleep(0.02)
    with lock:
        quiet = time.time() - vlm['recv']
    if quiet > 3:
        print('  no results from the laptop for %.0f s - is vlm_bridge.py running?' % quiet)
    return None


def vlm_find():
    arm_down()
    d = vlm_fresh()
    if d and d.get('found'):
        return 500, 0.15
    say('Looking around with the arm (laptop vision)')
    for base, reach in SCAN:
        ik_move(reach, HOVER_Z, base, 800)
        pause(1.0)
        d = vlm_fresh()
        if d and d.get('found'):
            return base, reach
    return None


def vlm_track(base, reach):
    """Step by step: wait for a fresh laptop result, move the gripper toward the object, repeat."""
    say('Lining the gripper up (laptop vision)')
    x_dis, y_dis = float(base), float(reach)
    sx, sy, last, ok, misses = 1, 1, None, 0, 0
    for step in range(VLM_STEPS):
        if rospy.is_shutdown():
            break
        d = vlm_fresh()
        if d is None or not d.get('found'):
            misses += 1
            if misses >= 3:
                return 'lost', None
            continue
        misses = 0
        cx, cy = d['cx'], d['cy']
        ex, ey = cx - CENTRE_X, cy - CENTRE_Y
        print('  laptop sees %s at (%d, %d) conf %.2f -> arm: base %d, reach %.3f m'
              % (d.get('label', '?'), cx, cy, d.get('conf', 0), x_dis, y_dis))
        if abs(ex) < ON_TARGET and abs(ey) < ON_TARGET:
            ok += 1
            if ok >= 2:
                t = ik_move(y_dis, HOVER_Z, int(x_dis), 300)
                if t:
                    print('  above the object: base %d, reach %.3f m' % (x_dis, y_dis))
                    return 'ok', (int(x_dis), y_dis, t[2])
            continue
        ok = 0
        if last is not None:                    # moved the wrong way last step? reverse that axis
            if abs(last[0]) > ON_TARGET and abs(ex) > abs(last[0]) + 10:
                sx = -sx
                print('  sideways direction reversed')
            if abs(last[1]) > ON_TARGET and abs(ey) > abs(last[1]) + 10:
                sy = -sy
                print('  reach direction reversed')
        last = (ex, ey)
        x_dis = clamp(x_dis - sx * VLM_X_GAIN * ex, 200, 800)
        y_new = y_dis - sy * VLM_Y_GAIN * ey
        if y_new > MAX_REACH and abs(ey) > ON_TARGET:
            return 'far', None
        if y_new < MIN_REACH and abs(ey) > ON_TARGET:
            return 'near', None
        y_dis = clamp(y_new, MIN_REACH, MAX_REACH)
        if not ik_move(y_dis, HOVER_Z, int(x_dis), 400):
            return 'far', None
        pause(0.4)
    return 'lost', None


def still_there_vlm(base, reach):
    ik_move(reach, HOVER_Z, base, 1500)
    pause(1.8)
    d = vlm_fresh()
    return bool(d and d.get('found') and abs(d['cx'] - CENTRE_X) < 80 and abs(d['cy'] - CENTRE_Y) < 80)


def calib_grab():
    """Measure where the cube appears in the picture when it is exactly between the fingers.
    Uses the same arm moves as a real grab, so every offset is included."""
    reach, base = 0.15, 500
    say('Grab calibration: moving to the grab position')
    t = ik_move(reach, HOVER_Z, base, 1500, extra=((1, GRIP_OPEN), (2, 500)))
    pause(1.6)
    if not t:
        print('IK failed - cannot calibrate')
        return
    offset = -0.04 + (t[2] + 180) * (0.07 / 30.0)       # same reach correction as grab()
    ik_move(reach + offset, GRAB_Z, base, 1500)
    pause(1.8)
    input('Slide the blue cube exactly between the open fingers (centred, as if grabbed). Press Enter: ')
    ik_move(reach, HOVER_Z, base, 1500)
    pause(2.0)
    hsv_pts, vlm_pts = [], []
    tr, seq, end = Tracker(), latest['seq'], time.time() + 6
    while time.time() < end and len(hsv_pts) < 15:
        seq, strict, loose, _ = next_frame(seq)
        pos, fresh = tr.update(strict, loose)
        if fresh:
            hsv_pts.append(pos)
    with lock:
        seen = vlm['count']
    end = time.time() + 6
    while time.time() < end and len(vlm_pts) < 5:
        with lock:
            d, c = vlm['msg'], vlm['count']
        if c != seen and d and d.get('found'):
            seen = c
            vlm_pts.append((d['cx'], d['cy']))
        time.sleep(0.05)
    try:
        with open(POSE_FILE) as f:
            data = json.load(f)
    except (IOError, OSError, ValueError):
        data = {}
    med = lambda pts, i: sorted(p[i] for p in pts)[len(pts) // 2]
    if hsv_pts:
        data['grab_centre'] = [int(med(hsv_pts, 0)), int(med(hsv_pts, 1))]
        print('Saved GRAB CENTRE (colour detection) at %s - was (%d, %d)' % (data['grab_centre'], CENTRE_X, CENTRE_Y))
    else:
        print('The robot did not see the blue cube - nothing saved for colour detection')
    if vlm_pts:
        data['grab_centre_vlm'] = [int(med(vlm_pts, 0)), int(med(vlm_pts, 1))]
        print('Saved GRAB CENTRE (laptop vision) at %s' % data['grab_centre_vlm'])
    else:
        print('No laptop results - start vlm_bridge.py on the laptop to calibrate laptop vision too')
    with open(POSE_FILE, 'w') as f:
        json.dump(data, f, indent=1)


# ---------------- front camera: ACEBOTT K210 module on the chassis ----------------
FRONT_PORT = '/dev/ttyUSB0'
FRONT_W, FRONT_H = 320, 240
FRONT_BLUE = (12, 85, -25, 35, -75, -12)   # LAB colour range of the blue cube: L min/max, A min/max, B min/max
FRONT_RED = (20, 80, 30, 80, 0, 60)   # LAB colour range of the red cube
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
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = self.port, 115200, 0.1
        s.dtr = False                       # do not hold the module's reset/boot pins
        s.rts = False
        s.open()
        self.ser = s
        print('  waiting for the module to start (about 6 s)...')
        time.sleep(6)                       # let ACEBOTT's program finish starting
        s.reset_input_buffer()
        got, end = b'', time.time() + 8     # stop it: Ctrl-C until we see the >>> prompt
        while b'>>>' not in got and time.time() < end:
            s.write(b'\x03')
            time.sleep(0.3)
            got += s.read(2000)
        if b'>>>' not in got:
            raise IOError('the module did not stop its own program (no >>> prompt)')
        time.sleep(0.5)
        s.read(4000)                        # wait for the >>> prompt, then drop the rest
        s.write(b'\r\x01')                  # raw REPL
        time.sleep(0.5)
        resp = s.read(4000)
        code = K210_SCRIPT.format(vflip=FRONT_VFLIP, hmirror=FRONT_HMIRROR,
                                  th=', '.join(str(tuple(t)) for t in [FRONT_BLUE] + ([FRONT_RED] if PICK_RED else [])),
                                  px=FRONT_MIN_PIXELS).encode()
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


class FrontTopic(object):
    """Reads the front camera's detections from front_cam.py (topic /front_cam/target)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.det, self.t, self.count = None, 0.0, 0
        rospy.Subscriber('/front_cam/target', String, self.on_msg, queue_size=5)

    def on_msg(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        det = (d['cx'], d['cy'], d['w'], d['h'], d['pixels']) if d.get('found') else None
        with self.lock:
            self.det, self.t = det, time.time()
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


def front_point():
    """Wait until the front camera sees the blue cube steadily; returns its detection or None."""
    say('Front camera: looking for a blue or red cube' if PICK_RED else 'Front camera: looking for the blue cube')
    seq, seen, t0 = front.count, 0, time.time()
    while not rospy.is_shutdown() and time.time() - t0 < 30:
        seq, det = front.latest(seq)
        if det:
            seen += 1
            if seen >= 3:
                print('  front camera sees the blue cube at (%d, %d), %d pixels' % (det[0], det[1], det[4]))
                return det
        else:
            seen = 0
    return None


def order_scan(cx):
    """Look first on the side where the front camera saw the cube (left = low base values)."""
    global SCAN
    right = cx > FRONT_W / 2.0
    SCAN = sorted(SCAN, key=lambda p: (0 if (p[0] > 500) == right else 1, abs(p[0] - 500), p[1]))


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


def main():
    global joints_pub, chassis_pub, image_pub, buzzer_pub, CENTRE_X, CENTRE_Y, front
    rospy.init_node('fetch_blue')
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    chassis_pub = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    image_pub = rospy.Publisher('/cleanup/image', Image, queue_size=1)
    buzzer_pub = rospy.Publisher('/sensor/buzzer', Float32, queue_size=1)
    rospy.on_shutdown(stop)
    signal.signal(signal.SIGHUP, lambda *a: rospy.signal_shutdown('hangup'))
    raw = {}
    try:
        with open(POSE_FILE) as f:
            raw = json.load(f)
        for k, v in raw.items():
            if isinstance(v, dict):
                poses[k] = dict((int(i), int(p)) for i, p in v.items())
    except (IOError, OSError, ValueError):
        pass
    gc = raw.get('grab_centre_vlm' if MODE == 'vlm' else 'grab_centre') or raw.get('grab_centre')
    if gc and MODE != 'calibgrab':
        CENTRE_X, CENTRE_Y = int(gc[0]), int(gc[1])
        print('Using calibrated grab point (%d, %d)' % (CENTRE_X, CENTRE_Y))
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, on_image, queue_size=1, buff_size=2 ** 24)
    if MODE in ('vlm', 'calibgrab'):
        rospy.Subscriber(VLM_TOPIC, String, on_vlm, queue_size=1)
    if MODE in ('front', 'frontlook', 'frontarm'):
        front = FrontTopic()
        print('Waiting for the front camera (front_cam.py must be running in another window)...')
        t_wait = time.time()
        while front.count == 0 and time.time() - t_wait < 25 and not rospy.is_shutdown():
            time.sleep(0.2)
        if front.count == 0:
            print('No results from the front camera - start it first in another window: python3 ~/front_cam.py')
            return
        print('Front camera running.')
    rospy.sleep(1.0)
    if latest['seq'] == 0:
        print('No camera pictures - is the camera running?')
        return
    print('Camera view: http://ubuntu.local:8080/stream_viewer?topic=/cleanup/image')
    if MODE == 'look':
        look_test()
        return
    if MODE == 'calibgrab':
        calib_grab()
        return
    if MODE == 'frontlook':
        front_look()
        return
    if 'look' not in poses or 'drop' not in poses:
        print('Teach the LOOK (key 1) and DROP (key 4) poses first: python3 ~/cleanup.py teach')
        return
    if MODE == 'vlm':
        print('Laptop vision mode: run vlm_bridge.py on the laptop. The wheels will not move. Ctrl+C stops.')
    elif ARM_ONLY:
        print('Arm only: the wheels will not move. Put the cube 12-25 cm in front. Ctrl+C stops.')
    else:
        print('This mode DRIVES. Use it on the floor. Press Ctrl+C to stop.')
    talk('Hello! I am looking for cubes to put in my basket.', INFO)
    if not ARM_ONLY:
        mv = battery_mv()
        if mv is not None:
            print('Battery: %.2f V' % (mv / 1000.0))
            if mv < MIN_BATTERY:
                print('Battery too low for driving (need at least %.1f V). Charge it first.' % (MIN_BATTERY / 1000.0))
                return
    placed, attempt = 0, 0
    while attempt < MAX_TRIES:
        attempt += 1
        if rospy.is_shutdown():
            return
        print('--- Attempt %d of %d ---' % (attempt, MAX_TRIES))
        if MODE == 'frontarm':
            det = front_point()
            if not det:
                say('Front camera sees no cube - checking with the arm camera')
                if not arm_find():
                    if placed:
                        break
                    say('The front camera does not see a cube - place one in front of the robot')
                    time.sleep(3)
                    continue
            else:
                order_scan(det[0])
        if MODE == 'front':
            arm_carry()
            if not front_search():
                if placed:
                    break
                say('The front camera sees no cube')
                return
            if not front_approach():
                continue
        elif not ARM_ONLY:
            arm_look()
            if not visible() and not search():
                if placed:
                    break
                say('No cube found anywhere around the robot')
                return
            if not approach():
                continue
        above = line_up()
        if not above:
            if placed and MODE in ('arm', 'vlm'):
                break
            continue
        base, reach, pitch = above
        wrist = 500
        if WRIST_ALIGN:
            wrist = align_wrist()
            if wrist != 500:                                  # re-centre after turning the wrist
                res, data = track(base, reach)
                if res == 'ok':
                    base, reach, pitch = data
        if not pick_up(base, reach, pitch, wrist):
            continue
        place_left()
        if still_there(base, reach):
            say('The cube is still there - the grab missed, trying again')
            continue
        placed += 1
        attempt = 0
        say('Cube %d is in the basket - looking for the next one' % placed)
        continue
    stop()
    arm_look()
    if placed:
        say('Finished: %d cube(s) in the basket, no more cubes found' % placed)
    else:
        say('Gave up after %d attempts' % MAX_TRIES)


if __name__ == '__main__':
    main()
