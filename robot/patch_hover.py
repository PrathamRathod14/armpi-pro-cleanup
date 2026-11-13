import sys
path = sys.argv[1] if len(sys.argv) > 1 else '/home/ubuntu/cleanup.py'
s = open(path).read()
code = '''BASE_GAIN = 0.2            # base servo units per pixel of error (far view)
BASE_TOL = 15              # pixels: how well centred in the far view
REACH_DY = 60              # pixels: how far the cube may be from the taught distance
BASE_TRIM = 0              # optional fixed sideways shift; leave 0 when the close view is calibrated
MAX_TRIES = 10             # attempts before giving up
GRIP_SQUEEZE = 60          # extra closing used to test whether the cube is held
HOLD_MARGIN = 25           # an empty gripper closes to within this of the test value
CLOSE_DIR = 1              # +1: higher gripper value = more closed (true on this robot)
USE_GRIP_CHECK = False     # squeeze test after lifting; off: it gave false 'empty' results
MISS_DY = 60               # cube still this close to the grab spot after placing = the grab missed
HOVER_FRAC = 0.6           # close view pose: this far from LOOK toward GRIP (0 = look, 1 = grip)
FINE_GAIN = 0.08           # base servo units per pixel of error (close view)
FINE_TOL = 8               # pixels: final accuracy in the close view

def nearest_blue(tx, ty):
    v = view()
    if not v or not v['objects']:
        return None
    return min(v['objects'], key=lambda o: (o[2] - tx) ** 2 + (o[3] - ty) ** 2)

def holding():
    """Squeeze a little harder: a gripper holding the cube is blocked, an empty one closes further."""
    g = poses['grip'][1]
    test = max(0, min(1000, g + CLOSE_DIR * GRIP_SQUEEZE))
    servo_pub.publish(servo_cls([item_cls(1, test, 400)]))
    time.sleep(0.8)
    p = read_positions().get(1)
    servo_pub.publish(servo_cls([item_cls(1, g, 300)]))
    time.sleep(0.4)
    if p is None:
        print('  grip check: cannot read the gripper, assuming it holds')
        return True
    held = abs(test - p) > HOLD_MARGIN
    print('  grip check: gripper at %d, empty would reach %d -> %s' % (p, test, 'HOLDING' if held else 'EMPTY'))
    return held

def hover_pose(gbase):
    """Arm partway down toward the grip pose, gripper open, so the camera sees the cube up close."""
    h = {}
    for j in (2, 3, 4, 5):
        a, b = poses['look'][j], poses['grip'][j]
        h[j] = int(round(a + HOVER_FRAC * (b - a)))
    h[6] = gbase
    h[1] = poses['ready'][1]
    return h

def calib2():
    missing = [n for n in ['look', 'ready', 'grip'] if n not in poses]
    if missing:
        print('Teach these first: ' + ', '.join(missing))
        return
    input('Put the blue cube exactly where the gripper grabs it (the pick-test spot), then press Enter: ')
    move(hover_pose(poses['grip'][6]), 2500)
    time.sleep(1.0)
    pts = []
    t0 = time.time()
    while time.time() - t0 < 1.5:
        tgt = target(view(), 'object')
        if tgt:
            pts.append((tgt[2], tgt[3]))
        time.sleep(0.1)
    move(arm('look', 'open'), 2500)
    if len(pts) < 5:
        print('Cube not seen in the close view. Set HOVER_FRAC lower (for example 0.4) and run calib2 again.')
        return
    xs = sorted(q[0] for q in pts)
    ys = sorted(q[1] for q in pts)
    poses['hover_target'] = [xs[len(xs) // 2], ys[len(ys) // 2]]
    save_poses()
    print('Saved HOVER TARGET at %s' % poses['hover_target'])

def reach_once():
    tx, ty = poses['look_target']
    base = poses['look'][6]
    move(arm('look', 'open'), 2500)
    time.sleep(0.8)
    say('Turning the arm to the cube')
    sign, last_dx, tgt = 1, None, None
    for step in range(30):
        tgt = nearest_blue(tx, ty)
        if not tgt:
            time.sleep(0.6)
            tgt = nearest_blue(tx, ty)
        if not tgt:
            say('No blue cube in view - put it in front of the robot, within reach')
            return 'wait'
        dx = tgt[2] - tx
        print('  far view: cube at (%d, %d), %d px off sideways, base %d' % (tgt[2], tgt[3], dx, base))
        if abs(dx) < BASE_TOL:
            break
        if last_dx is not None and abs(dx) > abs(last_dx) + 5:
            sign = -sign                     # the arm turned the wrong way: reverse
        last_dx = dx
        base = int(max(0, min(1000, base - sign * BASE_GAIN * dx)))
        move({6: base}, 400)
        time.sleep(0.4)
    else:
        say('Could not centre the cube')
        return 'wait'
    dy = tgt[3] - ty
    if abs(dy) > REACH_DY:
        say('Cube is %s: move it %s (about %d px off)'
            % ('too close' if dy > 0 else 'too far', 'away' if dy > 0 else 'closer', abs(dy)))
        return 'wait'
    off = base - poses['look'][6]
    gbase = int(max(0, min(1000, poses['grip'][6] + off + BASE_TRIM)))
    if 'hover_target' in poses:
        hx, hy = poses['hover_target']
        say('Close look: lining the gripper up exactly')
        move(hover_pose(gbase), 2000)
        time.sleep(0.8)
        fsign, last = 1, None
        for step in range(20):
            t2 = nearest_blue(hx, hy)
            if not t2:
                time.sleep(0.5)
                t2 = nearest_blue(hx, hy)
            if not t2:
                say('Lost the cube in the close view - trying again')
                return 'retry'
            ddx = t2[2] - hx
            print('  close view: cube at (%d, %d), %d px off sideways, base %d' % (t2[2], t2[3], ddx, gbase))
            if abs(ddx) < FINE_TOL:
                break
            if last is not None and abs(ddx) > abs(last) + 3:
                fsign = -fsign
            last = ddx
            gbase = int(max(0, min(1000, gbase - sign * fsign * FINE_GAIN * ddx)))
            move({6: gbase}, 300)
            time.sleep(0.4)
    else:
        print('  (no close-view calibration yet - run: python3 cleanup.py calib2)')
    say('Grabbing')
    g = arm('grip', 'open')
    g[6] = gbase
    move(g, 2000)
    move({1: poses['grip'][1]}, 1000)
    up = arm('look', 'closed')
    up[6] = gbase
    move(up, 2500)
    if USE_GRIP_CHECK and not holding():
        say('Gripper is empty - trying again')
        move({1: poses['ready'][1]}, 800)
        return 'retry'
    say('Placing on the left')
    move({6: poses['drop'][6]}, 2500)
    move(arm('drop', 'closed'), 2500)
    move({1: poses['ready'][1]}, 1000)
    up = arm('look', 'open')
    up[6] = poses['drop'][6]
    move(up, 2500)
    back = arm('look', 'open')
    back[6] = base                           # look back at where the cube was
    move(back, 2500)
    time.sleep(1.0)
    still = nearest_blue(tx, ty)
    if still and abs(still[2] - tx) < 3 * BASE_TOL + 30 and abs(still[3] - ty) < MISS_DY:
        say('The cube is still in its old place - the grab missed, trying again')
        return 'retry'
    move(arm('look', 'open'), 2500)
    return 'done'

def reach():
    missing = [n for n in ['look', 'ready', 'grip', 'drop', 'look_target'] if n not in poses]
    if missing:
        print('Not ready. Missing: ' + ', '.join(missing))
        return
    stop()                                   # wheels never move in this mode
    for attempt in range(1, MAX_TRIES + 1):
        if rospy.is_shutdown():
            return
        print('--- Attempt %d of %d ---' % (attempt, MAX_TRIES))
        result = reach_once()
        if result == 'done':
            say('Done: cube placed on the left')
            return
        if result == 'wait':
            time.sleep(3.0)                  # time to move the cube back into reach
    move(arm('look', 'open'), 2500)
    say('Gave up after %d attempts' % MAX_TRIES)

'''
marker = '# ---------------- teach mode ----------------'
if 'HOVER_FRAC' in s:
    print('already added')
elif 'BASE_GAIN = 0.2' not in s or 'def reach():' not in s:
    print('reach mode missing - add it first; nothing changed')
else:
    a = s.index('BASE_GAIN = 0.2')
    b = s.index(marker)
    s = s[:a] + code + s[b:]
    s = s.replace("'calib', 'fetch', 'reach'):", "'calib', 'fetch', 'reach', 'calib2'):")
    s = s.replace("    elif mode == 'calib':\n        calib()\n",
                  "    elif mode == 'calib':\n        calib()\n    elif mode == 'calib2':\n        calib2()\n")
    s = s.replace("'blue':   [((95, 110, 60), (130, 255, 255))]", "'blue':   [((95, 110, 120), (130, 255, 255))]")
    open(path, 'w').write(s)
    print('added')
