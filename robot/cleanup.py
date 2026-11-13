#!/usr/bin/env python3
# Clean-up robot for the ArmPi Pro.
#   python3 cleanup.py teach   -> record arm poses and reference points
#   python3 cleanup.py run     -> find objects, pick them up, place them on the robot's left
import sys, os, json, math, time, select, termios, tty, signal, threading
import numpy as np
import cv2
import rospy, rostopic, roslib.message
from sensor_msgs.msg import CompressedImage, Image

CHASSIS_TOPIC = '/chassis_control/set_velocity'
SERVO_TOPIC = '/servo_controllers/port_id_1/multi_id_pos_dur'
STATE_TOPIC = '/servo_controllers/port_id_1/servo_states'
CAM_TOPIC = '/usb_cam/image_raw/compressed'
POSE_FILE = os.path.expanduser('~/cleanup_poses.json')

# ---------------- settings you may need to tune ----------------
OBJECT_COLOURS = ['red', 'pink', 'yellow']
BOX_COLOUR = 'black'        # or 'green' / 'blue' if you wrap the holder in coloured paper
COLOURS = {                 # HSV ranges, hue 0-179
    'red':    [((0, 120, 70), (8, 255, 255)), ((174, 120, 70), (179, 255, 255))],
    'pink':   [((150, 70, 100), (173, 255, 255))],
    'yellow': [((20, 70, 90), (36, 255, 255))],
    'green':  [((40, 80, 60), (85, 255, 255))],
    'blue':   [((95, 110, 60), (130, 255, 255))],
    'black':  [((0, 0, 0), (179, 255, 60))],
}
MIN_OBJ_AREA = 500          # smallest object blob, in pixels
MIN_BOX_AREA = 2500         # smallest box blob, in pixels
BOX_IGNORE_BELOW = 0.80     # ignore the bottom of the image when looking for the box (gripper is there)
DRIVE_SPEED = 70.0          # mm/s when approaching
FINE_SPEED = 35.0           # mm/s for the final line-up
TURN_RATE = 0.25            # rotation rate
TURN_SIGN = 1               # set to -1 if the robot turns AWAY from what it sees
FLIP_X = False              # set True if the final line-up slides the wrong way sideways
NEAR_Y = 0.78               # object is "close" when its bottom edge passes this fraction of image height
CENTRE_TOL = 0.20           # how well centred before driving forward (fraction of half width)
FINE_TOL = 18               # final line-up tolerance in pixels
SEARCH_TIME = 45            # seconds of rotating before giving up
# ----------------------------------------------------------------

DRAW = {'red': (0, 0, 255), 'pink': (180, 105, 255), 'yellow': (0, 255, 255), 'box': (255, 255, 255)}
POSE_NAMES = ['look', 'ready', 'grip', 'drop']
lock = threading.Lock()
latest = {'t': 0.0, 'objects': [], 'box': None, 'size': (640, 480)}
state = 'starting'
box_filter = False
poses = {}
last_proc = 0.0
chassis_pub = servo_pub = image_pub = servo_cls = item_cls = None

# ---------------- vision ----------------
def colour_mask(hsv, name):
    mask = None
    for lo, hi in COLOURS[name]:
        m = cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask

def blobs(mask, name, min_area):
    out = []
    for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
        a = cv2.contourArea(c)
        if a >= min_area:
            x, y, w, h = cv2.boundingRect(c)
            out.append((a, name, x + w // 2, y + h // 2, (x, y, w, h)))
    return out

def detect(img):
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
    k = np.ones((5, 5), np.uint8)
    objects = []
    for name in OBJECT_COLOURS:
        objects += blobs(cv2.morphologyEx(colour_mask(hsv, name), cv2.MORPH_OPEN, k), name, MIN_OBJ_AREA)
    bm = colour_mask(hsv, BOX_COLOUR)
    bm[int(h * BOX_IGNORE_BELOW):, :] = 0
    bm = cv2.morphologyEx(bm, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    bm = cv2.morphologyEx(bm, cv2.MORPH_OPEN, k)
    found = sorted(blobs(bm, 'box', MIN_BOX_AREA), reverse=True)
    box = found[0] if found else None
    if box and box_filter:      # ignore objects that are already in the box
        x, y, bw, bh = box[4]
        mx, my = bw * 0.2, bh * 0.2
        objects = [o for o in objects
                   if not (x - mx <= o[2] <= x + bw + mx and y - my <= o[3] <= y + bh + my)]
    return sorted(objects, reverse=True), box

def on_image(msg):
    global last_proc
    if time.time() - last_proc < 0.1:
        return
    last_proc = time.time()
    img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return
    objects, box = detect(img)
    h, w = img.shape[:2]
    with lock:
        latest.update(t=time.time(), objects=objects, box=box, size=(w, h))
    for a, name, cx, cy, (x, y, bw, bh) in objects + ([box] if box else []):
        cv2.rectangle(img, (x, y), (x + bw, y + bh), DRAW[name], 2)
        cv2.putText(img, name, (x, max(15, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, DRAW[name], 2)
    if 'grab_spot' in poses:
        gx, gy = poses['grab_spot']
        cv2.drawMarker(img, (int(gx), int(gy)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
    cv2.putText(img, state, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    out = Image()
    out.header = msg.header
    out.height, out.width, out.encoding, out.step = h, w, 'bgr8', w * 3
    out.data = img.tobytes()
    image_pub.publish(out)

def view():
    with lock:
        v = dict(latest)
    return v if time.time() - v['t'] < 0.6 else None

def target(v, kind):
    if v is None:
        return None
    if kind == 'box':
        return v['box']
    return v['objects'][0] if v['objects'] else None

# ---------------- motion ----------------
def drive(vel, direction, angular):
    chassis_pub.publish(float(vel), float(direction), float(angular))

def stop():
    drive(0, 0, 0)

def move(targets, ms):
    servo_pub.publish(servo_cls([item_cls(int(i), int(p), int(ms)) for i, p in sorted(targets.items())]))
    time.sleep(ms / 1000.0 + 0.3)

def arm(name, gripper):
    t = dict((i, p) for i, p in poses[name].items() if i != 1)
    t[1] = poses['ready'][1] if gripper == 'open' else poses['grip'][1]
    return t

def say(text):
    global state
    state = text
    print(text)

def search(kind):
    say('Searching for ' + kind)
    t0, seen = time.time(), 0
    while not rospy.is_shutdown() and time.time() - t0 < SEARCH_TIME:
        if target(view(), kind):
            stop()
            seen += 1
            if seen >= 3:
                return True
        else:
            seen = 0
            drive(0, 90, TURN_SIGN * TURN_RATE)
        time.sleep(0.1)
    stop()
    return False

def approach(kind):
    say('Approaching ' + kind)
    t0 = lost = time.time()
    while not rospy.is_shutdown() and time.time() - t0 < 60:
        v = view()
        tgt = target(v, kind)
        if not tgt:
            stop()
            if time.time() - lost > 2.0:
                return False
            time.sleep(0.1)
            continue
        lost = time.time()
        w, h = v['size']
        a, name, cx, cy, (x, y, bw, bh) = tgt
        if kind == 'box':
            aim_x, close = poses['box_stop'][0], (y + bh) >= poses['box_stop'][1] - 5
        else:
            aim_x, close = w / 2.0, (y + bh) >= NEAR_Y * h
        ex = (cx - aim_x) / (w / 2.0)
        if close and abs(ex) < CENTRE_TOL:
            stop()
            return True
        if abs(ex) > CENTRE_TOL or close:
            drive(0, 90, -TURN_SIGN * TURN_RATE * (1 if ex > 0 else -1))
        else:
            drive(DRIVE_SPEED, 90, -TURN_SIGN * TURN_RATE * ex)
        time.sleep(0.1)
    stop()
    return False

def fine_align():
    say('Lining up on object')
    gx, gy = poses['grab_spot']
    t0 = seen_at = time.time()
    ok = 0
    while not rospy.is_shutdown() and time.time() - t0 < 25:
        v = view()
        objs = v['objects'] if v else []
        if not objs:
            if time.time() - seen_at > 4:
                break
            drive(FINE_SPEED, 90, 0)      # creep forward until it comes into view
            time.sleep(0.1)
            continue
        seen_at = time.time()
        a, name, cx, cy, rect = min(objs, key=lambda o: (o[2] - gx) ** 2 + (o[3] - gy) ** 2)
        dx, dy = cx - gx, cy - gy
        if FLIP_X:
            dx = -dx
        if abs(dx) < FINE_TOL and abs(dy) < FINE_TOL:
            stop()
            ok += 1
            if ok >= 3:
                return True
        else:
            ok = 0
            drive(FINE_SPEED, math.degrees(math.atan2(-dy, dx)) % 360, 0)
        time.sleep(0.1)
    stop()
    return False

def run():
    global box_filter
    box_filter = False
    max_objects = 3
    missing = [n for n in ['look', 'ready', 'grip', 'drop', 'grab_spot'] if n not in poses]
    if missing:
        print('Not ready. Run "python3 cleanup.py teach" first. Missing: ' + ', '.join(missing))
        return
    count = 0
    while not rospy.is_shutdown() and count < max_objects:
        move(arm('look', 'open'), 1500)
        time.sleep(0.5)
        if not search('object'):
            break
        if not approach('object'):
            continue
        move(arm('ready', 'open'), 1500)
        time.sleep(0.5)
        if not fine_align():
            continue
        say('Picking up')
        move(arm('grip', 'open'), 1500)
        move({1: poses['grip'][1]}, 800)
        move(arm('look', 'closed'), 1500)
        say('Placing on the left')
        move({6: poses['drop'][6]}, 1500)
        move(arm('drop', 'closed'), 1500)
        move({1: poses['ready'][1]}, 700)
        up = arm('look', 'open')
        up[6] = poses['drop'][6]
        move(up, 1500)
        count += 1
    stop()
    move(arm('look', 'open'), 1500)
    say('Finished. Objects moved: %d' % count)

# ---------------- teach mode ----------------
DRIVE_KEYS = {'w': (60.0, 90.0, 0.0), 's': (60.0, 270.0, 0.0), 'a': (60.0, 180.0, 0.0),
              'd': (60.0, 0.0, 0.0), 'q': (0.0, 90.0, 0.3), 'e': (0.0, 90.0, -0.3)}
ARM_KEYS = {'j': (6, 1), 'l': (6, -1), 'i': (5, 1), 'k': (5, -1), 'u': (4, 1), 'o': (4, -1),
            'y': (3, 1), 'h': (3, -1), 'n': (2, 1), 'm': (2, -1), 'r': (1, 1), 'f': (1, -1)}
TEACH_HELP = """
Move with the usual keys (w/s/a/d/q/e drive, j/l i/k u/o y/h n/m arm, r/f gripper).
  1  save LOOK pose   (arm raised, camera looking ahead along the floor, gripper open)
  2  save READY pose  (arm pointing down, gripper OPEN, object visible between the fingers)
  3  save GRIP pose   (arm lowered onto the object, gripper CLOSED on it)
  4  save DROP pose   (arm turned LEFT and lowered, object resting on the floor)
  c  save GRAB SPOT   (in READY pose, with an object sitting exactly where the gripper takes it)
  b  save BOX STOP    (in LOOK pose, robot parked where the DROP pose reaches over the box)
  v  show what is saved and what the camera sees        x  quit
"""

def read_positions():
    cls = rostopic.get_topic_class(STATE_TOPIC)[0]
    if cls is None:
        return {}
    try:
        msg = rospy.wait_for_message(STATE_TOPIC, cls, timeout=3.0)
    except rospy.ROSException:
        return {}
    for slot in msg.__slots__:
        val = getattr(msg, slot)
        if isinstance(val, (list, tuple)) and val and hasattr(val[0], 'id') and hasattr(val[0], 'position'):
            return dict((int(s.id), int(s.position)) for s in val)
    return {}

def save_poses():
    data = {}
    for k, v in poses.items():
        data[k] = dict((str(i), p) for i, p in v.items()) if isinstance(v, dict) else v
    with open(POSE_FILE, 'w') as f:
        json.dump(data, f, indent=1)

def load_poses():
    try:
        with open(POSE_FILE) as f:
            data = json.load(f)
    except (IOError, OSError, ValueError):
        return
    for k, v in data.items():
        poses[k] = dict((int(i), int(p)) for i, p in v.items()) if isinstance(v, dict) else v

def teach():
    global state
    state = 'teach'
    pos = read_positions()
    if len(pos) < 6:
        print('Could not read all six servo positions (got %s). Cannot teach.' % sorted(pos.items()))
        return
    print(TEACH_HELP)
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    current, last_key, last_arm = (0.0, 0.0, 0.0), 0.0, 0.0
    try:
        while not rospy.is_shutdown():
            cmd = current
            if select.select([sys.stdin], [], [], 0.05)[0]:
                k = sys.stdin.read(1).lower()
                now = time.time()
                if k == 'x':
                    break
                if k in '1234':
                    name = POSE_NAMES[int(k) - 1]
                    poses[name] = dict(pos)
                    save_poses()
                    print('Saved %s pose: %s' % (name.upper(), sorted(pos.items())))
                elif k == 'c':
                    tgt = target(view(), 'object')
                    if tgt:
                        poses['grab_spot'] = [tgt[2], tgt[3]]
                        save_poses()
                        print('Saved GRAB SPOT at %s (%s object)' % (poses['grab_spot'], tgt[1]))
                    else:
                        print('No object seen - cannot save grab spot')
                elif k == 'b':
                    tgt = target(view(), 'box')
                    if tgt:
                        x, y, bw, bh = tgt[4]
                        poses['box_stop'] = [tgt[2], y + bh]
                        save_poses()
                        print('Saved BOX STOP at %s' % poses['box_stop'])
                    else:
                        print('Box not seen - cannot save box stop')
                elif k == 'v':
                    v = view()
                    print('Saved: %s' % ', '.join(sorted(poses.keys())))
                    print('Objects seen: %s' % ([(o[1], o[2], o[3]) for o in v['objects']] if v else 'no image'))
                    print('Box seen: %s' % str(v['box'][4] if v and v['box'] else 'no'))
                elif k in ARM_KEYS:
                    cmd = (0.0, 0.0, 0.0)
                    if now - last_arm >= 0.05:
                        sid, sign = ARM_KEYS[k]
                        pos[sid] = max(0, min(1000, pos[sid] + sign * 8))
                        servo_pub.publish(servo_cls([item_cls(sid, pos[sid], 70)]))
                        last_arm = now
                else:
                    cmd = DRIVE_KEYS.get(k, (0.0, 0.0, 0.0))
                    last_key = now
            elif time.time() - last_key > 0.6:
                cmd = (0.0, 0.0, 0.0)
            if cmd != current:
                chassis_pub.publish(*cmd)
                current = cmd
    finally:
        stop()
        time.sleep(0.2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)

def main():
    global chassis_pub, servo_pub, image_pub, servo_cls, item_cls
    mode = sys.argv[1] if len(sys.argv) > 1 else ''
    if mode not in ('teach', 'run'):
        print('Usage: python3 cleanup.py teach    or    python3 cleanup.py run')
        return
    rospy.init_node('cleanup')
    chassis_cls = rostopic.get_topic_class(CHASSIS_TOPIC)[0]
    servo_cls = rostopic.get_topic_class(SERVO_TOPIC)[0]
    if chassis_cls is None or servo_cls is None:
        print('Cannot find the chassis or servo topic - is the robot stack running?')
        return
    item_cls = roslib.message.get_message_class(servo_cls._slot_types[0].replace('[]', ''))
    chassis_pub = rospy.Publisher(CHASSIS_TOPIC, chassis_cls, queue_size=1)
    servo_pub = rospy.Publisher(SERVO_TOPIC, servo_cls, queue_size=1)
    image_pub = rospy.Publisher('/cleanup/image', Image, queue_size=1)
    rospy.on_shutdown(stop)
    signal.signal(signal.SIGHUP, lambda *a: rospy.signal_shutdown('hangup'))
    load_poses()
    rospy.Subscriber(CAM_TOPIC, CompressedImage, on_image, queue_size=1, buff_size=2 ** 24)
    time.sleep(1.0)
    print('Camera view: http://ubuntu.local:8080/stream_viewer?topic=/cleanup/image')
    if mode == 'teach':
        teach()
    else:
        print('Running. Press Ctrl+C to stop at any time.')
        run()

if __name__ == '__main__':
    main()
