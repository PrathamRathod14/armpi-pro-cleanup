"""Arrange before grabbing (needs patch_holdcheck.py first).

1. Fine centring: right before going down, the arm centres itself over the cube much more
   exactly than the first line-up (within 8 pixels instead of 20), so a cube that is a bit
   left or right of the fingers is corrected first.
2. Settle grip: the gripper closes once so the fingers push the cube into the middle,
   loosens a little so the cube sits straight, then closes firmly.

Usage on the robot:  python3 ~/patch_arrange.py   (patches ~/fetch_blue.py)
"""
import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'def arrange(' in s:
    print('already patched')
    sys.exit(0)
if 'def holding(' not in s:
    print('apply patch_holdcheck.py first')
    sys.exit(0)

ARRANGE = '''def arrange(base, reach, pitch):
    """Final careful centring right above the cube, so the fingers come down evenly on both sides."""
    say('Arranging: centring the gripper exactly over the cube')
    res, data = track(base, reach, fine=True)
    if res == 'ok':
        return data
    print('  fine centring did not settle - using the first line-up')
    return base, reach, pitch


'''

R = [
    ("GRAB_TRIES = 3 ",
     "FINE_TARGET, FINE_DEADBAND = 8, 4   # exact centring before grabbing (pixels)\n"
     "SETTLE_OPEN = 300          # loosen to this after the first close so the cube settles in the middle\n"
     "GRAB_TRIES = 3 "),
    # track(): a fine mode with a tighter target and a time limit
    ("def track(base, reach):\n", "def track(base, reach, fine=False):\n"),
    ("    say('Lining the gripper up over the cube')\n",
     "    if not fine:\n        say('Lining the gripper up over the cube')\n"
     "    tol, db, limit = (FINE_TARGET, FINE_DEADBAND, 12) if fine else (ON_TARGET, DEADBAND, 40)\n"),
    ("    while not rospy.is_shutdown() and time.time() - t0 < 40:\n",
     "    while not rospy.is_shutdown() and time.time() - t0 < limit:\n"),
    ("        dx = X_GAIN * (CENTRE_X - cx) if abs(cx - CENTRE_X) >= DEADBAND else 0.0\n"
     "        dy = Y_GAIN * (CENTRE_Y - cy) if abs(cy - CENTRE_Y) >= DEADBAND else 0.0\n",
     "        dx = X_GAIN * (CENTRE_X - cx) if abs(cx - CENTRE_X) >= db else 0.0\n"
     "        dy = Y_GAIN * (CENTRE_Y - cy) if abs(cy - CENTRE_Y) >= db else 0.0\n"),
    ("        if abs(cx - CENTRE_X) < ON_TARGET and abs(cy - CENTRE_Y) < ON_TARGET and t:\n",
     "        if abs(cx - CENTRE_X) < tol and abs(cy - CENTRE_Y) < tol and t:\n"),
    # pick_up(): arrange right before every grab
    ("            ref = cube_seen()\n        if not grab(base, reach, pitch, wrist):\n",
     "        base, reach, pitch = arrange(base, reach, pitch)\n        ref = cube_seen()\n"
     "        if not grab(base, reach, pitch, wrist):\n"),
    ("def pick_up(", ARRANGE + "def pick_up("),
    # grab(): close, loosen, close firmly
    ("    servos(500, ((1, GRIP_CLOSE),))\n    pause(0.8)\n    servos(300, ((1, GRIP_CLOSE + GRIP_EXTRA),))",
     "    servos(700, ((1, GRIP_CLOSE),))                      # first close: the fingers push the cube to the middle\n"
     "    pause(0.9)\n"
     "    servos(300, ((1, SETTLE_OPEN),))                     # loosen a little so the cube settles straight\n"
     "    pause(0.5)\n"
     "    servos(500, ((1, GRIP_CLOSE),))\n    pause(0.8)\n    servos(300, ((1, GRIP_CLOSE + GRIP_EXTRA),))"),
]
missing = [old[:60] for old, new in R if old not in s]
if missing:
    print('nothing changed, could not find:', missing)
    sys.exit(0)
for old, new in R:
    s = s.replace(old, new, 1)
open(path, 'w').write(s)
print('patched')
