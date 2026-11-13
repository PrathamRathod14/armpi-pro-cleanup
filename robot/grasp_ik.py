#!/usr/bin/env python3
# IK grasp of the blue cube, based on Hiwonder's intelligent_grasp method.
# The arm points the camera straight down, finds the blue cube, and moves itself
# above it with inverse kinematics (left/right AND near/far). Then it lowers the
# gripper straight down, grabs, places the cube on the robot's left, and checks.
# The wheels never move.    Run:  python3 ~/grasp_ik.py     Stop: Ctrl+C
import os, json, time, threading
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from armpi_pro import bus_servo_control
from kinematics import ik_transform

# ---------------- settings ----------------
BLUE = ((95, 110, 120), (130, 255, 255))   # HSV range of the blue cube
MIN_AREA = 300                  # smallest blue patch counted as the cube (pixels)
CENTRE_X, CENTRE_Y = 320, 410   # where the cube appears when the gripper is right above it (Hiwonder's value)
HOVER_Z = 0.03                  # gripper height while tracking (metres)
GRAB_Z = -0.08                  # gripper height when grabbing (metres, below the arm base)
GRIP_OPEN, GRIP_CLOSE = 120, 450   # low = open on this robot
X_GAIN, Y_GAIN = 0.03, 0.000015  # tracking gains (Hiwonder's values)
DEADBAND = 10                   # pixels: close enough to the target point
ON_TARGET = 20                  # pixels: must be this close to the cross before grabbing
MAX_REACH = 0.25                # metres: farthest the gripper can still grab at table height
JUMP = 140                      # pixels: a bigger jump between frames means a different object
STABLE_FRAMES = 10              # frames in a row on target before grabbing
MAX_TRIES = 10
SLOW = 2.0                      # 1 = original speed, 2 = half speed, 3 = a third
TRACK_MS = 120                  # duration of each small tracking move (ms)
POSE_FILE = os.path.expanduser('~/cleanup_poses.json')
SCAN = [(500, 0.15), (400, 0.15), (600, 0.15), (300, 0.15), (700, 0.15),
        (500, 0.22), (400, 0.22), (600, 0.22), (300, 0.22), (700, 0.22)]   # (base, reach) to look around
# ------------------------------------------

ik = ik_transform.ArmIK()
lock = threading.Lock()
latest = {'seq': 0, 'cube': None, 'size': (640, 480)}
status = 'starting'
last_proc = 0.0
joints_pub = image_pub = buzzer_pub = None


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def on_image(msg):
    global last_proc
    if time.time() - last_proc < 0.06:
        return
    last_proc = time.time()
    img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(BLUE[0]), np.array(BLUE[1]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    blobs = []
    for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
        a = cv2.contourArea(c)
        if a >= MIN_AREA:
            m = cv2.moments(c)
            cx, cy = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
            blobs.append((cx, cy, a, cv2.boundingRect(c)))
    blobs.sort(key=lambda b: -b[2])
    cube = blobs[0] if blobs else None
    with lock:
        latest['seq'] += 1
        latest['cube'] = cube
        latest['blobs'] = blobs
        latest['size'] = (w, h)
    if cube:
        x, y, bw, bh = cube[3]
        cv2.rectangle(img, (x, y), (x + bw, y + bh), (255, 0, 0), 2)
        cv2.putText(img, 'blue', (x, max(15, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    cv2.drawMarker(img, (CENTRE_X, CENTRE_Y), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
    cv2.putText(img, status, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    out = Image()
    out.header = msg.header
    out.height, out.width, out.encoding, out.step = h, w, 'bgr8', w * 3
    out.data = img.tobytes()
    image_pub.publish(out)


def pause(sec):
    time.sleep(sec * SLOW)


def say(text):
    global status
    status = text
    print(text)


def servos(ms, pairs):
    bus_servo_control.set_servos(joints_pub, int(ms * SLOW), tuple((int(i), int(p)) for i, p in pairs))


def ik_move(reach, z, base, ms, extra=()):
    """Put the gripper (pointing straight down) at `reach` metres ahead and height z, base turned to `base`."""
    t = ik.setPitchRanges((0, round(reach, 4), z), -180, -180, 0)
    if not t:
        return None
    sd = t[1]
    servos(ms, tuple(extra) + ((3, sd['servo3']), (4, sd['servo4']), (5, sd['servo5']), (6, base)))
    return t


def cube_now():
    with lock:
        return latest['seq'], latest['cube']


def wait_frames(n=3):
    s0 = cube_now()[0]
    t0 = time.time()
    while cube_now()[0] < s0 + n and time.time() - t0 < 2:
        time.sleep(0.02)
    return cube_now()[1]


def home():
    ik_move(0.15, HOVER_Z, 500, 1500, extra=((1, GRIP_OPEN), (2, 500)))
    pause(1.8)


def look_around():
    """Point the camera at a few places until the blue cube is in view."""
    say('Looking for the blue cube')
    for base, reach in SCAN:
        if rospy.is_shutdown():
            return None
        ik_move(reach, HOVER_Z, base, 800)
        pause(1.0)
        if wait_frames(3):
            return base, reach
    return None


def track(base, reach):
    """Keep the cube on the target point using IK, until it stays there."""
    say('Tracking the cube')
    x_dis, y_dis = float(base), float(reach)
    last_seq, stable, t_lost, t0 = cube_now()[0], 0, time.time(), time.time()
    shown, prev, at_limit = 0.0, None, 0
    while not rospy.is_shutdown() and time.time() - t0 < 40:
        seq, cube = cube_now()
        if seq == last_seq:
            time.sleep(0.01)
            continue
        last_seq = seq
        if prev is not None:                 # follow the same object, not whatever is biggest
            with lock:
                blobs = list(latest.get('blobs', []))
            near = [b for b in blobs if (b[0] - prev[0]) ** 2 + (b[1] - prev[1]) ** 2 < JUMP ** 2]
            cube = min(near, key=lambda b: (b[0] - prev[0]) ** 2 + (b[1] - prev[1]) ** 2) if near else None
        if cube is None:
            stable = 0
            if time.time() - t_lost > 3:
                say('Lost the cube')
                return None
            continue
        t_lost = time.time()
        cx, cy = cube[0], cube[1]
        prev = (cx, cy)
        dx = X_GAIN * (CENTRE_X - cx) if abs(cx - CENTRE_X) >= DEADBAND else 0.0
        dy = Y_GAIN * (CENTRE_Y - cy) if abs(cy - CENTRE_Y) >= DEADBAND else 0.0
        x_dis = clamp(x_dis + dx, 200, 800)
        y_dis = clamp(y_dis + dy, 0.12, MAX_REACH)
        t = ik_move(y_dis, HOVER_Z, int(x_dis), TRACK_MS / SLOW)
        if time.time() - shown > 0.5:
            shown = time.time()
            print('  cube in picture (%d, %d) -> arm: base %d, reach %.3f m' % (cx, cy, x_dis, y_dis))
        if y_dis >= MAX_REACH and cy < CENTRE_Y - ON_TARGET:
            at_limit += 1
            if at_limit >= 15:
                say('Object is beyond reach - move it closer (12-25 cm ahead)')
                return None
        else:
            at_limit = 0
        if abs(cx - CENTRE_X) < ON_TARGET and abs(cy - CENTRE_Y) < ON_TARGET and t:
            stable += 1
            if stable >= STABLE_FRAMES:
                print('  above the cube: base %d, reach %.3f m' % (x_dis, y_dis))
                return int(x_dis), y_dis, t[2]
        else:
            stable = 0
    say('Could not settle above the cube')
    return None


def grab(base, reach, pitch):
    say('Grabbing')
    offset = -0.04 + (pitch + 180) * (0.07 / 30.0)    # Hiwonder's reach correction for the achieved pitch
    buzzer_pub.publish(0.1)
    servos(500, ((1, GRIP_OPEN),))
    pause(0.5)
    if not ik_move(reach + offset, GRAB_Z, base, 1000):
        say('Cube is out of reach')
        return False
    pause(1.5)
    servos(500, ((1, GRIP_CLOSE),))
    pause(0.8)
    servos(1500, ((1, GRIP_CLOSE), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))   # lift (Hiwonder's carry pose)
    pause(1.6)
    return True


def place_left(drop):
    say('Placing on the left')
    servos(1200, ((6, drop[6]),))
    pause(1.3)
    servos(1500, ((3, drop[3]), (4, drop[4]), (5, drop[5])))
    pause(1.8)
    servos(500, ((1, GRIP_OPEN),))
    pause(0.8)
    servos(1200, ((3, 80), (4, 825), (5, 625)))
    pause(1.3)


def still_there(base, reach):
    """Look again where the cube was: if it is still under the camera, the grab missed."""
    ik_move(reach, HOVER_Z, base, 1500)
    pause(1.8)
    cube = wait_frames(3)
    return bool(cube and abs(cube[0] - CENTRE_X) < 80 and abs(cube[1] - CENTRE_Y) < 80)


def main():
    global joints_pub, image_pub, buzzer_pub
    rospy.init_node('grasp_ik')
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    image_pub = rospy.Publisher('/cleanup/image', Image, queue_size=1)
    buzzer_pub = rospy.Publisher('/sensor/buzzer', Float32, queue_size=1)
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, on_image, queue_size=1, buff_size=2 ** 24)
    try:
        with open(POSE_FILE) as f:
            drop = dict((int(k), int(v)) for k, v in json.load(f)['drop'].items())
    except (IOError, OSError, ValueError, KeyError):
        print('No taught DROP pose in %s - teach it first (cleanup.py teach, key 4).' % POSE_FILE)
        return
    rospy.sleep(0.5)
    print('Camera view: http://ubuntu.local:8080/stream_viewer?topic=/cleanup/image')
    print('Wheels never move. Press Ctrl+C to stop.')
    home()
    if not wait_frames(5) and latest['seq'] == 0:
        print('No camera pictures - is the camera running?')
        return
    for attempt in range(1, MAX_TRIES + 1):
        if rospy.is_shutdown():
            return
        print('--- Attempt %d of %d ---' % (attempt, MAX_TRIES))
        found = (500, 0.15) if wait_frames(3) else look_around()
        if not found:
            say('No blue cube within reach - put it in front of the robot, 12-28 cm ahead')
            home()
            time.sleep(3)
            continue
        above = track(*found)
        if not above:
            home()
            continue
        base, reach, pitch = above
        if not grab(base, reach, pitch):
            home()
            continue
        place_left(drop)
        if still_there(base, reach):
            say('The cube is still there - the grab missed, trying again')
            continue
        home()
        say('Done: cube placed on the left')
        return
    home()
    say('Gave up after %d attempts' % MAX_TRIES)


if __name__ == '__main__':
    main()
