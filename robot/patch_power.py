import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'MIN_BATTERY' in s:
    print('already patched')
    sys.exit(0)
R = [
    ("""def drive(vel, direction, angular):
    if ARM_ONLY and (vel or angular):
        return                                  # never move the wheels in arm / look mode
    chassis_pub.publish(float(vel), float(direction), float(angular))
""", """MIN_BATTERY = 7200          # millivolts: below this the robot will not drive (power dips restart the Pi)
RAMP_VEL, RAMP_ANG = 12.0, 0.06   # most the wheel speed / turn rate may rise per command (gentle starts)
last_drive = [0.0, 0.0]


def battery_mv():
    try:
        import rostopic
        cls = rostopic.get_topic_class('/sensor/battery')[0]
        return int(rospy.wait_for_message('/sensor/battery', cls, timeout=3.0).data)
    except Exception:
        return None


def drive(vel, direction, angular):
    if ARM_ONLY and (vel or angular):
        return                                  # never move the wheels in arm / look mode
    if vel == 0 and angular == 0:
        last_drive[0] = last_drive[1] = 0.0
    else:                                       # ramp up instead of jumping: smaller current spikes
        vel = min(vel, last_drive[0] + RAMP_VEL)
        if abs(angular) > last_drive[1] + RAMP_ANG:
            angular = (last_drive[1] + RAMP_ANG) * (1 if angular > 0 else -1)
        last_drive[0], last_drive[1] = vel, abs(angular)
    chassis_pub.publish(float(vel), float(direction), float(angular))
"""),
    ("    for attempt in range(1, MAX_TRIES + 1):\n        if rospy.is_shutdown():\n            return\n        print('--- Attempt",
     "    if not ARM_ONLY:\n"
     "        mv = battery_mv()\n"
     "        if mv is not None:\n"
     "            print('Battery: %.2f V' % (mv / 1000.0))\n"
     "            if mv < MIN_BATTERY:\n"
     "                print('Battery too low for driving (need at least %.1f V). Charge it first.' % (MIN_BATTERY / 1000.0))\n"
     "                return\n"
     "    for attempt in range(1, MAX_TRIES + 1):\n        if rospy.is_shutdown():\n            return\n        print('--- Attempt"),
]
missing = [old[:60] for old, new in R if old not in s]
if missing:
    print('nothing changed, could not find:', missing)
    sys.exit(0)
for old, new in R:
    s = s.replace(old, new, 1)
open(path, 'w').write(s)
print('patched')
