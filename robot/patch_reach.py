import sys
path = sys.argv[1] if len(sys.argv) > 1 else '/home/ubuntu/cleanup.py'
s = open(path).read()
code = '''BASE_GAIN = 0.2            # base servo units per pixel of error
BASE_TOL = 15              # pixels: how well centred before grabbing
REACH_DY = 60              # pixels: how far the cube may be from the taught distance

def reach():
    missing = [n for n in ['look', 'ready', 'grip', 'drop', 'look_target'] if n not in poses]
    if missing:
        print('Not ready. Missing: ' + ', '.join(missing))
        return
    stop()                                   # wheels never move in this mode
    tx, ty = poses['look_target']
    base = poses['look'][6]
    move(arm('look', 'open'), 2500)
    time.sleep(0.8)
    say('Turning the arm to the cube')
    sign, last_dx, tgt = 1, None, None
    for step in range(30):
        tgt = target(view(), 'object')
        if not tgt:
            time.sleep(0.6)
            tgt = target(view(), 'object')
        if not tgt:
            say('No blue cube in view - put it in front of the robot, within reach')
            return
        dx = tgt[2] - tx
        print('  cube at (%d, %d), %d px from the spot sideways, base %d' % (tgt[2], tgt[3], dx, base))
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
        return
    dy = tgt[3] - ty
    if abs(dy) > REACH_DY:
        say('Cube is %s: move it %s (about %d px off) and try again'
            % ('too close' if dy > 0 else 'too far', 'away' if dy > 0 else 'closer', abs(dy)))
        return
    off = base - poses['look'][6]
    say('Grabbing')
    g = arm('grip', 'open')
    g[6] = max(0, min(1000, poses['grip'][6] + off))
    move(g, 2000)
    move({1: poses['grip'][1]}, 1000)
    up = arm('look', 'closed')
    up[6] = base
    move(up, 2500)
    say('Placing on the left')
    move({6: poses['drop'][6]}, 2500)
    move(arm('drop', 'closed'), 2500)
    move({1: poses['ready'][1]}, 1000)
    up = arm('look', 'open')
    up[6] = poses['drop'][6]
    move(up, 2500)
    move(arm('look', 'open'), 2500)
    say('Done: cube placed on the left')

'''
marker = '# ---------------- teach mode ----------------'
if 'def reach(' in s:
    print('already added')
elif "elif mode == 'calib':" not in s:
    print('fetch patch missing - add it first; nothing changed')
else:
    s = s.replace(marker, code + marker)
    s = s.replace("'calib', 'fetch'):", "'calib', 'fetch', 'reach'):")
    s = s.replace("    elif mode == 'calib':\n        calib()\n",
                  "    elif mode == 'calib':\n        calib()\n    elif mode == 'reach':\n"
                  "        print('Arm only: the wheels will not move. Press Ctrl+C to stop.')\n        reach()\n")
    open(path, 'w').write(s)
    print('added')
