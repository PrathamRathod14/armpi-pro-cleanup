"""Smooth, safe driving for fetch_blue.py front (the robot restarted while turning).

What made it crash: with a blue and a red cube in view, the front camera reported whichever
was bigger, so the target jumped left/right, the robot flipped its turning direction, and the
wheels reversed instantly - a big current spike that dropped the battery voltage and restarted the Pi.

Fixes:
  1. The wheels never reverse instantly: speed and turning change gradually, through zero.
  2. The robot locks onto one cube and ignores a jump to another one.
  3. Turning is proportional (slower when nearly lined up) and slower overall.
  4. Lower top speeds.
Needs patch_gentle.py first.

Usage on the robot:  python3 ~/patch_navigate.py   (patches ~/fetch_blue.py)
"""
import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'LOCK_JUMP' in s:
    print('already patched')
    sys.exit(0)
if 'RAMP_VEL_S' not in s:
    print('apply patch_gentle.py first')
    sys.exit(0)

DRIVE = '''def drive(vel, direction, angular):
    """Send a wheel command, changing speed and turning gradually - never an instant reversal."""
    import math
    if ARM_ONLY and (vel or angular):
        return                                  # never move the wheels in arm / look mode
    now = time.time()
    if vel == 0 and angular == 0:
        last_drive[:] = [0.0, 0.0, 0.0, now]
        chassis_pub.publish(0.0, 90.0, 0.0)
        return
    dt = min(0.15, now - last_drive[3]) if last_drive[3] else 0.05
    tx, ty = vel * math.cos(math.radians(direction)), vel * math.sin(math.radians(direction))
    dx, dy = tx - last_drive[0], ty - last_drive[1]
    d, m = math.hypot(dx, dy), RAMP_VEL_S * dt
    if d > m:
        dx, dy = dx * m / d, dy * m / d
    vx, vy = last_drive[0] + dx, last_drive[1] + dy
    w = last_drive[2] + max(-RAMP_ANG_S * dt, min(RAMP_ANG_S * dt, angular - last_drive[2]))
    last_drive[:] = [vx, vy, w, now]
    v = math.hypot(vx, vy)
    chassis_pub.publish(float(v), float(math.degrees(math.atan2(vy, vx)) % 360 if v > 0.01 else 90.0), float(w))


def ease_off(sec=0.8):
    """Slow the wheels down gradually to a standstill."""
    t0 = time.time()
    while not rospy.is_shutdown() and time.time() - t0 < sec:
        drive(SOFT, 90, 0)
        time.sleep(0.05)
    stop()


'''

APPROACH = '''def front_approach():
    """Steer the wheels by the front camera until the cube is close enough for the arm.
    Follows one cube only, turns smoothly, and eases off instead of reversing suddenly."""
    global TURN_SIGN
    say('Driving to the cube (front camera)')
    seq, low, lock, jumps = front.count, False, None, 0
    last_err, t_check, t_lost, t0, shown = None, time.time(), time.time(), time.time(), 0.0
    while not rospy.is_shutdown() and time.time() - t0 < 90:
        seq, det = front.latest(seq)
        if det and lock is not None and abs(det[0] - lock[0]) > LOCK_JUMP and jumps < 8:
            jumps += 1                          # another cube is the biggest now: keep following ours
            continue
        if not det:
            if low:
                ease_off(0.4)
                print('  cube went under the front camera - close enough')
                return True
            drive(SOFT, 90, 0)                  # slow down gently while it is out of view
            if time.time() - t_lost > 2.0:
                ease_off(0.5)
                say('Lost the cube while driving')
                return False
            continue
        lock, jumps, t_lost = det, 0, time.time()
        cx, cy = det[0], det[1]
        low = cy > FRONT_UNDER * FRONT_H
        ex = (cx - FRONT_W / 2.0) / (FRONT_W / 2.0)
        if time.time() - shown > 0.5:
            shown = time.time()
            print('  front camera: cube at (%d, %d), size %d' % (cx, cy, det[4]))
        if cy >= FRONT_STOP_Y and abs(ex) < CENTRE_BAND:
            ease_off(0.5)
            return True
        turn = -TURN_SIGN * TURN_RATE * max(0.5, min(1.0, abs(ex) * 1.6)) * (1 if ex > 0 else -1)
        if abs(ex) > CENTRE_BAND:
            drive(SOFT, 90, turn)               # turn on the spot, faster the further off-centre
            if time.time() - t_check > 1.0:
                if last_err is not None and abs(ex) > last_err + 0.15:
                    print('  turning the wrong way - easing off, then turning the other way')
                    ease_off(0.8)
                    TURN_SIGN = -TURN_SIGN
                last_err, t_check = abs(ex), time.time()
        else:
            last_err = None
            drive(DRIVE_SPEED if cy < 0.6 * FRONT_H else CREEP_SPEED, 90, -TURN_SIGN * TURN_RATE * ex)   # gentle steering
    ease_off(0.5)
    return False


'''

R = [
    ("TURN_RATE = 0.25\n", "TURN_RATE = 0.15           # turning speed (lower = smaller current spikes)\n"),
    ("DRIVE_SPEED = 45.0 ", "DRIVE_SPEED = 35.0 "),
    ("CREEP_SPEED = 30.0 ", "CREEP_SPEED = 22.0 "),
    ("last_drive = [0.0, 0.0, 0.0]       # speed, turn rate, time of the last command\n",
     "last_drive = [0.0, 0.0, 0.0, 0.0]  # sideways speed, forward speed, turn rate, time of the last command\n"
     "SOFT = 0.001               # 'almost zero' speed: drive() eases down to it instead of stopping dead\n"
     "LOCK_JUMP = 70             # front camera: a jump of more than this (pixels) is another cube\n"),
    # front_search: ease off when the cube is seen, instead of stopping dead
    ("        seq, det = front.latest(seq)\n        if det:\n            stop()\n            seen += 1\n",
     "        seq, det = front.latest(seq)\n        if det:\n            drive(SOFT, 90, 0)\n            seen += 1\n"),
]
missing = [old[:60] for old, new in R if old not in s]
for a, b in (('def drive(', 'def stop():'), ('def front_approach():', 'def front_point():')):
    if a not in s or b not in s:
        missing.append(a)
if missing:
    print('nothing changed, could not find:', missing)
    sys.exit(0)
for old, new in R:
    s = s.replace(old, new, 1)
a, b = s.index('def drive('), s.index('def stop():')
s = s[:a] + DRIVE + s[b:]
a, b = s.index('def front_approach():'), s.index('def front_point():')
s = s[:a] + APPROACH + s[b:]
open(path, 'w').write(s)
print('patched')
