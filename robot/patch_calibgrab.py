import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'def calib_grab(' in s:
    print('already patched')
    sys.exit(0)

CALIB = '''def calib_grab():
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


'''

R = [
    ("ARM_ONLY = MODE in ('arm', 'look', 'vlm')\n", "ARM_ONLY = MODE in ('arm', 'look', 'vlm', 'calibgrab')\n"),
    ("def main():\n    global joints_pub, chassis_pub, image_pub, buzzer_pub\n",
     CALIB + "def main():\n    global joints_pub, chassis_pub, image_pub, buzzer_pub, CENTRE_X, CENTRE_Y\n"),
    ("""    try:
        with open(POSE_FILE) as f:
            for k, v in json.load(f).items():
                if isinstance(v, dict):
                    poses[k] = dict((int(i), int(p)) for i, p in v.items())
    except (IOError, OSError, ValueError):
        pass
""", """    raw = {}
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
"""),
    ("    if MODE == 'vlm':\n        rospy.Subscriber(VLM_TOPIC, String, on_vlm, queue_size=1)\n",
     "    if MODE in ('vlm', 'calibgrab'):\n        rospy.Subscriber(VLM_TOPIC, String, on_vlm, queue_size=1)\n"),
    ("    if MODE == 'look':\n        look_test()\n        return\n",
     "    if MODE == 'look':\n        look_test()\n        return\n"
     "    if MODE == 'calibgrab':\n        calib_grab()\n        return\n"),
]
missing = [old[:60] for old, new in R if old not in s]
if missing:
    print('nothing changed, could not find:', missing)
    sys.exit(0)
for old, new in R:
    s = s.replace(old, new, 1)
open(path, 'w').write(s)
print('patched')
