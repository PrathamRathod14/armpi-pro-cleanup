"""Keep fetch_blue.py front from crashing the robot (needs patch_gentle.py + patch_navigate.py).

wander.py drives fine with the front camera running, so the wheels alone are not the problem.
What fetch_blue.py does on top, and what this patch changes:
  1. Less work for the Pi while driving: the arm camera's pictures are not processed while the
     front camera steers, and the arm-camera picture with boxes is sent 3x less often.
  2. Battery watch: the battery is read all the time; if it dips below 7.0 V while driving,
     the wheels stop for 2 s so it can recover (instead of the Pi restarting).
  3. Nudges (creeping closer for the arm) ramp up gently too.
  4. The battery level is printed every 3 s, and how long the Pi has been on at the start,
     so a log shows whether a crash was a power dip or a Wi-Fi drop.

Usage on the robot:  python3 ~/patch_safedrive.py   (patches ~/fetch_blue.py)
"""
import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'SAG_MV' in s:
    print('already patched')
    sys.exit(0)
if 'LOCK_JUMP' not in s:
    print('apply patch_gentle.py and patch_navigate.py first')
    sys.exit(0)

HELPERS = '''SAG_MV = 7000              # battery below this while driving: stop the wheels and let it recover
SAG_REST = 2.0             # seconds to rest after a dip
VIEW_EVERY = 3             # send the arm-camera picture with boxes every 3rd frame (less CPU and Wi-Fi)
batt = {'mv': None, 't': 0.0, 'low': None, 'shown': 0.0}
arm_cam_paused = False     # arm-camera pictures are not processed while the front camera steers
view_count = [0]


def start_batt_watch():
    """Read the battery continuously; print it every 3 s with the lowest value seen."""
    try:
        import rostopic
        cls = rostopic.get_topic_class('/sensor/battery', blocking=False)[0]
        if cls is None:
            print('  (battery topic not found - no battery watch)')
            return

        def on_batt(m):
            mv = int(m.data)
            batt['mv'], batt['t'] = mv, time.time()
            batt['low'] = mv if batt['low'] is None else min(batt['low'], mv)
            if time.time() - batt['shown'] > 3.0:
                batt['shown'] = time.time()
                print('  [battery %.2f V, lowest %.2f V]' % (mv / 1000.0, batt['low'] / 1000.0), flush=True)
        rospy.Subscriber('/sensor/battery', cls, on_batt, queue_size=1)
        with open('/proc/uptime') as f:
            print('  (the Pi has been on for %.0f minutes)' % (float(f.read().split()[0]) / 60.0))
    except Exception as e:
        print('  (battery watch: %s)' % e)


def _no_arm_cam(fn):
    """Run a driving step with the arm camera's processing paused (the front camera steers)."""
    global arm_cam_paused
    arm_cam_paused = True
    try:
        return fn()
    finally:
        arm_cam_paused = False


'''

R = [
    # 1. arm camera: paused while driving, picture sent less often
    ("def on_image(msg):\n    global last_proc\n    if time.time() - last_proc < 0.06:\n",
     "def on_image(msg):\n    global last_proc\n    if arm_cam_paused:\n        return\n    if time.time() - last_proc < 0.06:\n"),
    ("    out = Image()\n    out.header = msg.header\n",
     "    view_count[0] += 1\n    if view_count[0] % VIEW_EVERY:\n        return\n    out = Image()\n    out.header = msg.header\n"),
    ("def front_search():\n", "def front_search():\n    return _no_arm_cam(_front_search)\n\n\ndef _front_search():\n"),
    ("def front_approach():\n", "def front_approach():\n    return _no_arm_cam(_front_approach)\n\n\ndef _front_approach():\n"),
    # 2. battery watch inside drive()
    ("    now = time.time()\n    if vel == 0 and angular == 0:\n        last_drive[:] = [0.0, 0.0, 0.0, now]\n",
     "    now = time.time()\n"
     "    if (vel > SOFT or angular) and batt['mv'] is not None and batt['mv'] < SAG_MV and now - batt['t'] < 2.0:\n"
     "        print('  battery dipped to %.2f V while driving - resting the wheels %.0f s' % (batt['mv'] / 1000.0, SAG_REST), flush=True)\n"
     "        chassis_pub.publish(0.0, 90.0, 0.0)\n"
     "        last_drive[:] = [0.0, 0.0, 0.0, now]\n"
     "        time.sleep(SAG_REST)\n"
     "        batt['mv'] = None\n"
     "        return\n"
     "    if vel == 0 and angular == 0:\n        last_drive[:] = [0.0, 0.0, 0.0, now]\n"),
    ("def drive(vel, direction, angular):\n", HELPERS + "def drive(vel, direction, angular):\n"),
    # 3. nudge ramps up instead of one single command
    ("def nudge(direction):\n    drive(CREEP_SPEED, direction, 0)\n    time.sleep(NUDGE_TIME)\n    stop()\n",
     "def nudge(direction):\n    t0 = time.time()\n    while not rospy.is_shutdown() and time.time() - t0 < NUDGE_TIME:\n"
     "        drive(CREEP_SPEED, direction, 0)\n        time.sleep(0.05)\n    ease_off(0.5)\n"),
    # 4. start the battery watch
    ("    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, on_image, queue_size=1, buff_size=2 ** 24)\n",
     "    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, on_image, queue_size=1, buff_size=2 ** 24)\n"
     "    start_batt_watch()\n"),
]
missing = [old[:60] for old, new in R if old not in s]
if missing:
    print('nothing changed, could not find:', missing)
    sys.exit(0)
for old, new in R:
    s = s.replace(old, new, 1)
open(path, 'w').write(s)
print('patched')
