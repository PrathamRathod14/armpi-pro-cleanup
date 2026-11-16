"""Gentler wheel starts in fetch_blue.py, like wander.py (which runs without the robot restarting).

Before: the speed could rise by 12 mm/s per command; commands come ~15 times a second,
so the wheels went from 0 to full speed in about 0.25 s - a big current spike.
After: the speed rises by time, 0 to 45 mm/s in about 0.9 s, and turning ramps up as slowly.

Usage on the robot:  python3 ~/patch_gentle.py   (patches ~/fetch_blue.py)
"""
import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'RAMP_VEL_S' in s:
    print('already patched')
    sys.exit(0)
R = [
    ("RAMP_VEL, RAMP_ANG = 12.0, 0.06   # most the wheel speed / turn rate may rise per command (gentle starts)\n"
     "last_drive = [0.0, 0.0]\n",
     "RAMP_VEL_S, RAMP_ANG_S = 50.0, 0.3   # most the wheel speed (mm/s) / turn rate may rise per SECOND\n"
     "last_drive = [0.0, 0.0, 0.0]       # speed, turn rate, time of the last command\n"),
    ("""    if vel == 0 and angular == 0:
        last_drive[0] = last_drive[1] = 0.0
    else:                                       # ramp up instead of jumping: smaller current spikes
        vel = min(vel, last_drive[0] + RAMP_VEL)
        if abs(angular) > last_drive[1] + RAMP_ANG:
            angular = (last_drive[1] + RAMP_ANG) * (1 if angular > 0 else -1)
        last_drive[0], last_drive[1] = vel, abs(angular)
""",
     """    now = time.time()
    if vel == 0 and angular == 0:
        last_drive[0] = last_drive[1] = 0.0
    else:                                       # ramp up slowly by time: small current spikes
        dt = min(0.15, now - last_drive[2]) if last_drive[2] else 0.05
        vel = min(vel, last_drive[0] + RAMP_VEL_S * dt)
        if abs(angular) > last_drive[1] + RAMP_ANG_S * dt:
            angular = (last_drive[1] + RAMP_ANG_S * dt) * (1 if angular > 0 else -1)
        last_drive[0], last_drive[1] = vel, abs(angular)
    last_drive[2] = now
"""),
]
missing = [old[:60] for old, new in R if old not in s]
if missing:
    print('nothing changed, could not find:', missing)
    sys.exit(0)
for old, new in R:
    s = s.replace(old, new, 1)
open(path, 'w').write(s)
print('patched')
