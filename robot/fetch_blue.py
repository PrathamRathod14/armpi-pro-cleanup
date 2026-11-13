#!/usr/bin/env python3
# Find the blue cube anywhere around the robot, drive to it, grab it with
# inverse kinematics, and place it on the robot's left. Retries until it succeeds.
#   python3 ~/fetch_blue.py          Stop: Ctrl+C (the wheels stop)
# Needs the taught LOOK and DROP poses in ~/cleanup_poses.json.
import os, json, time, threading, signal
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from chassis_control.msg import SetVelocity
from armpi_pro import bus_servo_control
from kinematics import ik_transform

# ---------------- detection ----------------
BLUE = ((95, 110, 120), (130, 255, 255))   # HSV range of the blue cube
MIN_AREA = 150             # smallest patch, in pixels (small = can see the cube from farther away)
MIN_SOLIDITY = 0.80        # patch must be solid, not ragged (rejects reflections and clothing folds)
MIN_FILL = 0.50            # patch must fill at least half of its box (rejects lines and edges)
MAX_ASPECT = 2.5           # patch may not be much longer than wide
CONFIRM = 3                # pictures in a row before a detection is believed
# ---------------- wheels ----------------
TURN_RATE = 0.25           # turning speed while searching and aiming
TURN_SIGN = 1              # reversed automatically if the robot turns away from the cube
DRIVE_SPEED = 45.0         # mm/s while the cube is far
CREEP_SPEED = 30.0         # mm/s when close
CENTRE_BAND = 0.15         # how centred the cube must be before driving (fraction of half width)
NEAR_STOP = 0.82           # stop driving when the cube's centre is this low in the picture
SEARCH_TIME = 60           # seconds of turning before giving up
NUDGE_TIME = 1.2           # seconds of creeping when the arm cannot reach
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
# ---------------------------------------

ik = ik_transform.ArmIK()
lock = threading.Lock()
latest = {'seq': 0, 'blobs': [], 'size': (640, 480)}
status = 'starting'
last_proc = 0.0
poses = {}
joints_pub = chassis_pub = image_pub = buzzer_pub = None


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------------- vision ----------------
def detect(img):
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(BLUE[0]), np.array(BLUE[1]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    blobs = []
    for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
        a = cv2.contourArea(c)
        if a < MIN_AREA:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        hull_area = cv2.contourArea(cv2.convexHull(c))
        solidity = a / hull_area if hull_area > 0 else 0
        fill = a / float(bw * bh)
        aspect = max(bw, bh) / float(max(1, min(bw, bh)))
        if solidity < MIN_SOLIDITY or fill < MIN_FILL or aspect > MAX_ASPECT:
            continue
        m = cv2.moments(c)
        blobs.append((int(m['m10'] / m['m00']), int(m['m01'] / m['m00']), a, (x, y, bw, bh)))
    blobs.sort(key=lambda b: -b[2])
    return blobs


def on_image(msg):
    global last_proc
    if time.time() - last_proc < 0.06:
        return
    last_proc = time.time()
    img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return
    h, w = img.shape[:2]
    blobs = detect(img)
    with lock:
        latest['seq'] += 1
        latest['blobs'] = blobs
        latest['size'] = (w, h)
    for i, (cx, cy, a, (x, y, bw, bh)) in enumerate(blobs):
        cv2.rectangle(img, (x, y), (x + bw, y + bh), (255, 0, 0), 3 if i == 0 else 1)
        if i == 0:
            cv2.putText(img, 'blue cube', (x, max(15, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
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
                return latest['seq'], list(latest['blobs']), latest['size']
        time.sleep(0.01)
    with lock:
        return latest['seq'], [], latest['size']


def follow(prev, blobs):
    """Keep following the same cube: the blob nearest its last position (or the biggest at first)."""
    if not blobs:
        return None
    if prev is None:
        return blobs[0]
    near = [b for b in blobs if (b[0] - prev[0]) ** 2 + (b[1] - prev[1]) ** 2 < JUMP ** 2]
    return min(near, key=lambda b: (b[0] - prev[0]) ** 2 + (b[1] - prev[1]) ** 2) if near else None


def visible(n=3):
    """True if a cube is seen in n pictures in a row."""
    seq = latest['seq']
    for _ in range(n):
        seq, blobs, _ = next_frame(seq)
        if not blobs:
            return None
    return blobs[0]


# ---------------- motion ----------------
def say(text):
    global status
    status = text
    print(text)


def pause(sec):
    time.sleep(sec * SLOW)


def drive(vel, direction, angular):
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
    """Arm raised, camera looking ahead across the floor (the taught LOOK pose)."""
    lp = poses['look']
    servos(1500, ((1, GRIP_OPEN), (2, lp[2]), (3, lp[3]), (4, lp[4]), (5, lp[5]), (6, lp[6])))
    pause(1.6)


def arm_down():
    """Gripper pointing down, camera looking at the floor just in front of the robot."""
    ik_move(0.15, HOVER_Z, 500, 1500, extra=((1, GRIP_OPEN), (2, 500)))
    pause(1.6)


def nudge(direction):
    """Creep forward (90) or backward (270) a little, with the arm still."""
    drive(CREEP_SPEED, direction, 0)
    time.sleep(NUDGE_TIME)
    stop()
    time.sleep(0.5)


# ---------------- 1. find and drive (wheels) ----------------
def search():
    say('Searching: turning to find the blue cube')
    seq, seen, t0 = latest['seq'], 0, time.time()
    while not rospy.is_shutdown() and time.time() - t0 < SEARCH_TIME:
        seq, blobs, _ = next_frame(seq)
        if blobs:
            stop()
            seen += 1
            if seen >= CONFIRM:
                return True
        else:
            seen = 0
            drive(0, 90, TURN_SIGN * TURN_RATE)
    stop()
    return False


def approach():
    global TURN_SIGN
    say('Driving to the cube')
    seq, prev = latest['seq'], None
    t_lost = t_check = time.time()
    last_err = None
    t0 = time.time()
    while not rospy.is_shutdown() and time.time() - t0 < 90:
        seq, blobs, (w, h) = next_frame(seq)
        cube = follow(prev, blobs)
        if cube is None:
            stop()
            if time.time() - t_lost > 2.0:
                say('Lost the cube while driving')
                return False
            continue
        t_lost = time.time()
        prev = (cube[0], cube[1])
        ex = (cube[0] - w / 2.0) / (w / 2.0)
        if cube[1] >= NEAR_STOP * h and abs(ex) < CENTRE_BAND:
            stop()
            return True
        if abs(ex) > CENTRE_BAND:
            drive(0, 90, -TURN_SIGN * TURN_RATE * (1 if ex > 0 else -1))
            if time.time() - t_check > 0.8:            # turning the wrong way? reverse
                if last_err is not None and abs(ex) > last_err + 0.08:
                    TURN_SIGN = -TURN_SIGN
                    print('  turning the other way')
                last_err, t_check = abs(ex), time.time()
        else:
            last_err = None
            speed = DRIVE_SPEED if cube[1] < 0.6 * h else CREEP_SPEED
            drive(speed, 90, -TURN_SIGN * TURN_RATE * 0.5 * ex)
    stop()
    return False


# ---------------- 2. line the gripper up (arm, inverse kinematics) ----------------
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
    """Move the gripper over the cube with IK. Returns ('ok', (base, reach, pitch)) or 'far' / 'near' / 'lost'."""
    say('Lining the gripper up over the cube')
    x_dis, y_dis = float(base), float(reach)
    seq, prev, stable, far, near = latest['seq'], None, 0, 0, 0
    t_lost = t0 = shown = time.time()
    while not rospy.is_shutdown() and time.time() - t0 < 40:
        seq, blobs, _ = next_frame(seq)
        cube = follow(prev, blobs)
        if cube is None:
            stable = 0
            if time.time() - t_lost > 3:
                return 'lost', None
            continue
        t_lost = time.time()
        cx, cy = cube[0], cube[1]
        prev = (cx, cy)
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
    """Arm-only line-up, creeping the wheels a little if the cube is out of the arm's reach."""
    for adjust in range(4):
        found = arm_find()
        if not found:
            result = 'far' if adjust % 2 == 0 else 'near'
        else:
            result, data = track(*found)
            if result == 'ok':
                return data
        if result == 'far':
            say('Cube a little too far for the arm - creeping forward')
            nudge(90)
        elif result == 'near':
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
    cube = visible()
    return bool(cube and abs(cube[0] - CENTRE_X) < 80 and abs(cube[1] - CENTRE_Y) < 80)


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
    if 'look' not in poses or 'drop' not in poses:
        print('Teach the LOOK (key 1) and DROP (key 4) poses first: python3 ~/cleanup.py teach')
        return
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, on_image, queue_size=1, buff_size=2 ** 24)
    rospy.sleep(1.0)
    if latest['seq'] == 0:
        print('No camera pictures - is the camera running?')
        return
    print('Camera view: http://ubuntu.local:8080/stream_viewer?topic=/cleanup/image')
    print('This mode DRIVES. Use it on the floor. Press Ctrl+C to stop.')
    for attempt in range(1, MAX_TRIES + 1):
        if rospy.is_shutdown():
            return
        print('--- Attempt %d of %d ---' % (attempt, MAX_TRIES))
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
