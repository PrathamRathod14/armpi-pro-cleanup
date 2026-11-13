import sys
path = sys.argv[1] if len(sys.argv) > 1 else '/home/ubuntu/cleanup.py'
s = open(path).read()
if 'def grip_for(' in s:
    print('already added')
    sys.exit(0)
if 'def hover_pose(gbase):' not in s:
    print('close-view patch missing - add it first; nothing changed')
    sys.exit(0)

helpers = '''def reach_t(y):
    """0 at the near grab spot, 1 at the far one, measured by the cube's height in the far view."""
    if 'look_target_far' not in poses or 'grip_far' not in poses:
        return 0.0
    yn, yf = poses['look_target'][1], poses['look_target_far'][1]
    return 0.0 if yf == yn else (y - yn) / float(yf - yn)

def aim_x(y):
    t = max(0.0, min(1.0, reach_t(y)))
    xn = poses['look_target'][0]
    xf = poses.get('look_target_far', poses['look_target'])[0]
    return xn + t * (xf - xn)

def grip_for(y):
    """Blend the near and far grip poses to match how far away the cube is."""
    if 'grip_far' not in poses:
        return dict(poses['grip'])
    t = max(-0.2, min(1.2, reach_t(y)))
    return dict((j, int(round(poses['grip'][j] + t * (poses['grip_far'][j] - poses['grip'][j]))))
                for j in poses['grip'])

def calibfar():
    if 'look' not in poses or 'grip_far' not in poses:
        print('Teach LOOK and GRIP FAR (key 5 in teach mode) first.')
        return
    move(arm('look', 'open'), 2500)
    time.sleep(1.0)
    input('Leave the blue cube at the FAR grab spot (where you taught key 5), then press Enter: ')
    pts = []
    t0 = time.time()
    while time.time() - t0 < 1.5:
        tgt = target(view(), 'object')
        if tgt:
            pts.append((tgt[2], tgt[3]))
        time.sleep(0.1)
    if len(pts) < 5:
        print('Cube not seen from the LOOK pose - nothing saved.')
        return
    xs = sorted(q[0] for q in pts)
    ys = sorted(q[1] for q in pts)
    poses['look_target_far'] = [xs[len(xs) // 2], ys[len(ys) // 2]]
    save_poses()
    print('Saved FAR LOOK TARGET at %s (near is %s)' % (poses['look_target_far'], poses['look_target']))

'''

def rep(old, new, scope=None):
    global s
    if scope:
        a = s.index(scope[0]); b = s.index(scope[1], a)
        seg = s[a:b]
        assert old in seg, 'not found: ' + old[:50]
        s = s[:a] + seg.replace(old, new, 1) + s[b:]
    else:
        assert old in s, 'not found: ' + old[:50]
        s = s.replace(old, new, 1)

try:
    R = ('def reach_once():', 'def reach():')
    rep("def hover_pose(gbase):", helpers + "def hover_pose(gbase, gp=None):")
    rep("        a, b = poses['look'][j], poses['grip'][j]", "        a, b = poses['look'][j], (gp or poses['grip'])[j]")
    rep("        dx = tgt[2] - tx\n", "        dx = int(tgt[2] - aim_x(tgt[3]))\n", R)
    rep("""    dy = tgt[3] - ty
    if abs(dy) > REACH_DY:
        say('Cube is %s: move it %s (about %d px off)'
            % ('too close' if dy > 0 else 'too far', 'away' if dy > 0 else 'closer', abs(dy)))
        return 'wait'
    off = base - poses['look'][6]
    gbase = int(max(0, min(1000, poses['grip'][6] + off + BASE_TRIM)))""",
        """    t = reach_t(tgt[3])
    gp = grip_for(tgt[3])
    if 'grip_far' in poses:
        print('  distance: %.2f  (0 = near grab spot, 1 = far grab spot)' % t)
        if t < -0.25 or t > 1.25:
            say('Cube is outside the taught reach: move it %s' % ('closer' if t > 1.25 else 'away'))
            return 'wait'
    else:
        dy = tgt[3] - ty
        if abs(dy) > REACH_DY:
            say('Cube is %s: move it %s (about %d px off)'
                % ('too close' if dy > 0 else 'too far', 'away' if dy > 0 else 'closer', abs(dy)))
            return 'wait'
    ox, oy = tgt[2], tgt[3]
    off = base - poses['look'][6]
    gbase = int(max(0, min(1000, gp[6] + off + BASE_TRIM)))""", R)
    rep("        move(hover_pose(gbase), 2000)", "        move(hover_pose(gbase, gp), 2000)", R)
    rep("""    g = arm('grip', 'open')
    g[6] = gbase
    move(g, 2000)
    move({1: poses['grip'][1]}, 1000)""", """    g = dict(gp)
    g[1] = poses['ready'][1]
    g[6] = gbase
    move(g, 2000)
    move({1: gp[1]}, 1000)""", R)
    rep("""    still = nearest_blue(tx, ty)
    if still and abs(still[2] - tx) < 3 * BASE_TOL + 30 and abs(still[3] - ty) < MISS_DY:""",
        """    still = nearest_blue(ox, oy)
    if still and abs(still[2] - ox) < 3 * BASE_TOL + 30 and abs(still[3] - oy) < MISS_DY:""", R)
    rep("                elif k == 'c':", """                elif k == '5':
                    poses['grip_far'] = dict(pos)
                    save_poses()
                    print('Saved GRIP FAR pose: %s' % sorted(pos.items()))
                elif k == 'c':""")
    rep("  c  save GRAB SPOT", "  5  save GRIP FAR   (cube placed farther away, gripper CLOSED on it)\n  c  save GRAB SPOT")
    rep("'reach', 'calib2'):", "'reach', 'calib2', 'calibfar'):")
    rep("    elif mode == 'calib2':\n        calib2()\n",
        "    elif mode == 'calib2':\n        calib2()\n    elif mode == 'calibfar':\n        calibfar()\n")
except (AssertionError, ValueError) as e:
    print('nothing changed: ' + str(e))
    sys.exit(0)
open(path, 'w').write(s)
print('added')
