import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'ARM_ONLY' in s:
    print('already patched')
    sys.exit(0)
R = [
    ("import os, json, time, threading, signal\n", "import os, sys, json, time, threading, signal\n"),
    ("MAX_TRIES = 8\n", "MAX_TRIES = 8\n"
     "ARM_ONLY = len(sys.argv) > 1 and sys.argv[1] == 'arm'   # 'python3 fetch_blue.py arm': the wheels never move\n"),
    ("def drive(vel, direction, angular):\n",
     "def drive(vel, direction, angular):\n"
     "    if ARM_ONLY and (vel or angular):\n"
     "        return                               # arm-only mode: never move the wheels\n"),
    ("""        if result == 'far':
            say('Cube a little too far for the arm - creeping forward')
            nudge(90)
        elif result == 'near':
            say('Cube too close for the arm - backing up a little')
            nudge(270)""",
     """        if result == 'far':
            if ARM_ONLY:
                say('Cube too far for the arm - move it a little closer')
                time.sleep(4)
            else:
                say('Cube a little too far for the arm - creeping forward')
                nudge(90)
        elif result == 'near':
            if ARM_ONLY:
                say('Cube too close for the arm - move it a little away')
                time.sleep(4)
            else:
                say('Cube too close for the arm - backing up a little')
                nudge(270)"""),
    ("    print('This mode DRIVES. Use it on the floor. Press Ctrl+C to stop.')\n",
     "    if ARM_ONLY:\n"
     "        print('Arm only: the wheels will not move. Put the cube 12-25 cm in front. Ctrl+C stops.')\n"
     "    else:\n"
     "        print('This mode DRIVES. Use it on the floor. Press Ctrl+C to stop.')\n"),
    ("""        arm_look()
        if not visible() and not search():
            say('No blue cube found anywhere around the robot')
            return
        if not approach():
            continue
""", """        if not ARM_ONLY:
            arm_look()
            if not visible() and not search():
                say('No blue cube found anywhere around the robot')
                return
            if not approach():
                continue
"""),
]
missing = [old[:50] for old, new in R if old not in s]
if missing:
    print('nothing changed, could not find:', missing)
    sys.exit(0)
for old, new in R:
    s = s.replace(old, new, 1)
open(path, 'w').write(s)
print('patched')
