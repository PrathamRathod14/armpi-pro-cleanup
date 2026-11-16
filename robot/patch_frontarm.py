import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'def front_point(' in s:
    print('already patched')
    sys.exit(0)

CODE = '''def front_point():
    """Wait until the front camera sees the blue cube steadily; returns its detection or None."""
    say('Front camera: looking for the blue cube')
    seq, seen, t0 = front.count, 0, time.time()
    while not rospy.is_shutdown() and time.time() - t0 < 30:
        seq, det = front.latest(seq)
        if det:
            seen += 1
            if seen >= 3:
                print('  front camera sees the blue cube at (%d, %d), %d pixels' % (det[0], det[1], det[4]))
                return det
        else:
            seen = 0
    return None


def order_scan(cx):
    """Look first on the side where the front camera saw the cube (left = low base values)."""
    global SCAN
    right = cx > FRONT_W / 2.0
    SCAN = sorted(SCAN, key=lambda p: (0 if (p[0] > 500) == right else 1, abs(p[0] - 500), p[1]))


'''

R = [
    ("ARM_ONLY = MODE in ('arm', 'look', 'vlm', 'calibgrab', 'frontlook')\n",
     "ARM_ONLY = MODE in ('arm', 'look', 'vlm', 'calibgrab', 'frontlook', 'frontarm')\n"),
    ("def front_look():\n", CODE + "def front_look():\n"),
    ("    if MODE in ('front', 'frontlook'):\n", "    if MODE in ('front', 'frontlook', 'frontarm'):\n"),
    ("        if MODE == 'front':\n            arm_carry()\n",
     "        if MODE == 'frontarm':\n"
     "            det = front_point()\n"
     "            if not det:\n"
     "                say('The front camera does not see the blue cube - place it in front of the robot')\n"
     "                time.sleep(3)\n"
     "                continue\n"
     "            order_scan(det[0])\n"
     "        if MODE == 'front':\n            arm_carry()\n"),
]
missing = [old[:60] for old, new in R if old not in s]
if missing:
    print('nothing changed, could not find:', missing)
    sys.exit(0)
for old, new in R:
    s = s.replace(old, new, 1)
open(path, 'w').write(s)
print('patched')
