import re, sys
path = sys.argv[1] if len(sys.argv) > 1 else '/home/ubuntu/cleanup.py'
s = open(path).read()
code = '''LOOK_TOL = 20               # pixels: how close to the taught spot counts as arrived

def calib():
    if 'look' not in poses:
        print('Teach the LOOK pose first.')
        return
    move(arm('look', 'open'), 2500)
    time.sleep(1.0)
    input('Put the blue cube exactly where the gripper picks it up (the pick-test spot), then press Enter: ')
    pts = []
    t0 = time.time()
    while time.time() - t0 < 1.5:
        tgt = target(view(), 'object')
        if tgt:
            pts.append((tgt[2], tgt[3]))
        time.sleep(0.1)
    if len(pts) < 5:
        print('Cube not seen from the LOOK pose. In teach mode tilt the wrist down a little, save LOOK (key 1), then run calib again.')
        return
    xs = sorted(q[0] for q in pts)
    ys = sorted(q[1] for q in pts)
    poses['look_target'] = [xs[len(xs) // 2], ys[len(ys) // 2]]
    save_poses()
    print('Saved LOOK TARGET at %s' % poses['look_target'])

def go_to_cube():
    say('Driving to the cube')
    tx, ty = poses['look_target']
    t0 = lost = time.time()
    ok = 0
    while not rospy.is_shutdown() and time.time() - t0 < 90:
        v = view()
        tgt = target(v, 'object')
        if not tgt:
            stop()
            if time.time() - lost > 2.0:
                return False
            time.sleep(0.1)
            continue
        lost = time.time()
        w, h = v['size']
        dx, dy = tgt[2] - tx, ty - tgt[3]      # dx > 0: cube right of the spot; dy > 0: cube further away
        if FLIP_X:
            dx = -dx
        if abs(dx) < LOOK_TOL and abs(dy) < LOOK_TOL:
            stop()
            ok += 1
            if ok >= 3:
                return True
            time.sleep(0.1)
            continue
        ok = 0
        if dy > 0.25 * h:                       # far away: turn to face it, then drive
            ex = dx / (w / 2.0)
            if abs(ex) > CENTRE_TOL:
                drive(0, 90, -TURN_SIGN * TURN_RATE * (1 if ex > 0 else -1))
            else:
                drive(DRIVE_SPEED, 90, -TURN_SIGN * TURN_RATE * ex)
        else:                                   # close: slide straight onto the spot
            drive(FINE_SPEED, math.degrees(math.atan2(dy, dx)) % 360, 0)
        time.sleep(0.1)
    stop()
    return False

def pick_and_place():
    say('Picking up')
    move(arm('grip', 'open'), 2000)
    move({1: poses['grip'][1]}, 1000)
    move(arm('look', 'closed'), 2500)
    say('Placing on the left')
    move({6: poses['drop'][6]}, 2500)
    move(arm('drop', 'closed'), 2500)
    move({1: poses['ready'][1]}, 1000)
    up = arm('look', 'open')
    up[6] = poses['drop'][6]
    move(up, 2500)
    move(arm('look', 'open'), 2500)

def fetch():
    missing = [n for n in ['look', 'ready', 'grip', 'drop', 'look_target'] if n not in poses]
    if missing:
        print('Not ready. Missing: ' + ', '.join(missing) + '   (look_target comes from: python3 cleanup.py calib)')
        return
    move(arm('look', 'open'), 2500)
    time.sleep(0.5)
    for attempt in range(3):
        if not search('object'):
            break
        if go_to_cube():
            pick_and_place()
            stop()
            say('Done: cube placed on the left')
            return
        print('Lost the cube - searching again')
    stop()
    say('Could not reach the cube')

'''
marker = '# ---------------- teach mode ----------------'
if 'def fetch(' in s:
    print('already added')
elif marker not in s or "elif mode == 'pick':" not in s:
    print('pick test missing - add it first; nothing changed')
else:
    s = s.replace(marker, code + marker)
    s = re.sub(r"^OBJECT_COLOURS = .*$", "OBJECT_COLOURS = ['blue']", s, flags=re.M)
    s = re.sub(r"^MIN_OBJ_AREA = \d+", "MIN_OBJ_AREA = 250", s, flags=re.M)
    s = s.replace("if mode not in ('teach', 'run', 'pick'):",
                  "if mode not in ('teach', 'run', 'pick', 'calib', 'fetch'):")
    s = s.replace("    elif mode == 'pick':\n        pick_test()\n",
                  "    elif mode == 'pick':\n        pick_test()\n    elif mode == 'calib':\n        calib()\n"
                  "    elif mode == 'fetch':\n        print('Fetching the blue cube. Press Ctrl+C to stop at any time.')\n        fetch()\n")
    open(path, 'w').write(s)
    print('added')
