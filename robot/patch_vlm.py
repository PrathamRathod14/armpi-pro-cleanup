import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'def vlm_track(' in s:
    print('already patched')
    sys.exit(0)

VLM_CODE = '''# ---------------- laptop vision (YOLO-World via /vlm/target) ----------------
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


'''

R = [
    ("from std_msgs.msg import Float32\n", "from std_msgs.msg import Float32, String\n"),
    ("ARM_ONLY = MODE in ('arm', 'look')\n", "ARM_ONLY = MODE in ('arm', 'look', 'vlm')\n"),
    ("def servos(ms, pairs):\n    bus_servo_control.set_servos(",
     "def servos(ms, pairs):\n    move_done[0] = time.time() + ms * SLOW / 1000.0\n    bus_servo_control.set_servos("),
    ("def arm_find():\n", "def arm_find():\n    if MODE == 'vlm':\n        return vlm_find()\n"),
    ("def track(base, reach):\n", "def track(base, reach):\n    if MODE == 'vlm':\n        return vlm_track(base, reach)\n"),
    ("def still_there(base, reach):\n",
     "def still_there(base, reach):\n    if MODE == 'vlm':\n        return still_there_vlm(base, reach)\n"),
    ("def main():\n", VLM_CODE + "def main():\n"),
    ("    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, on_image, queue_size=1, buff_size=2 ** 24)\n",
     "    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, on_image, queue_size=1, buff_size=2 ** 24)\n"
     "    if MODE == 'vlm':\n"
     "        rospy.Subscriber(VLM_TOPIC, String, on_vlm, queue_size=1)\n"),
    ("    if ARM_ONLY:\n        print('Arm only:",
     "    if MODE == 'vlm':\n"
     "        print('Laptop vision mode: run vlm_bridge.py on the laptop. The wheels will not move. Ctrl+C stops.')\n"
     "    elif ARM_ONLY:\n        print('Arm only:"),
]
missing = [old[:60] for old, new in R if old not in s]
if missing:
    print('nothing changed, could not find:', missing)
    sys.exit(0)
for old, new in R:
    s = s.replace(old, new, 1)
open(path, 'w').write(s)
print('patched')
