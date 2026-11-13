import os, re, sys
files = sys.argv[1:] or [os.path.expanduser('~/grasp_ik.py'), os.path.expanduser('~/grasp_black.py')]
SLOW_SETTINGS = ("SLOW = 2.0                      # 1 = original speed, 2 = half speed, 3 = a third\n"
                 "TRACK_MS = 120                  # duration of each small tracking move (ms)\n")
CAP_WORDS = [
    ("Looking for the blue cube", "Looking for the cap"),
    ("Tracking the cube", "Tracking the cap"),
    ("'  cube in picture (", "'  cap in picture ("),
    ("Lost the cube", "Lost the cap"),
    ("'  above the cube:", "'  above the cap:"),
    ("Could not settle above the cube", "Could not settle above the cap"),
    ("Cube is out of reach", "Cap is out of reach"),
    ("The cube is still there", "The cap is still there"),
    ("Done: cube placed on the left", "Done: cap placed on the left"),
    ("No blue cube within reach", "No cap within reach"),
    ("'black', (x, max(15, y - 6))", "'cap', (x, max(15, y - 6))"),
]
for path in files:
    try:
        s = open(path).read()
    except IOError:
        print(path, ': not found, skipped')
        continue
    changed = False
    if 'SLOW = ' not in s:
        anchor = "MAX_TRIES = 10\n"
        if anchor not in s or 'def servos(ms, pairs):' not in s:
            print(path, ': nothing changed, unexpected file')
            continue
        s = s.replace(anchor, anchor + SLOW_SETTINGS, 1)
        # all servo moves take SLOW times longer
        s = s.replace("bus_servo_control.set_servos(joints_pub, int(ms),",
                      "bus_servo_control.set_servos(joints_pub, int(ms * SLOW),", 1)
        s = s.replace("def say(text):", "def pause(sec):\n    time.sleep(sec * SLOW)\n\n\ndef say(text):", 1)
        # waits after moves get longer too (only inside the movement functions)
        for fn in ('def home(', 'def look_around(', 'def grab(', 'def place_left(', 'def still_there('):
            a = s.index(fn)
            b = s.index('\ndef ', a + 5)
            s = s[:a] + s[a:b].replace('time.sleep(', 'pause(') + s[b:]
        # gentler tracking: smaller steps, smoother moves
        s = s.replace("X_GAIN, Y_GAIN = 0.06, 0.00003", "X_GAIN, Y_GAIN = 0.03, 0.000015", 1)
        s = s.replace("t = ik_move(y_dis, HOVER_Z, int(x_dis), 20)", "t = ik_move(y_dis, HOVER_Z, int(x_dis), TRACK_MS / SLOW)", 1)
        changed = True
    if path.endswith('grasp_black.py'):
        for old, new in CAP_WORDS:
            if old in s:
                s = s.replace(old, new)
                changed = True
    if changed:
        open(path, 'w').write(s)
        print(path, ': patched')
    else:
        print(path, ': already patched')
