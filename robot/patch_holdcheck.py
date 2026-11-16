"""Only go to the basket once the cube is really in the gripper.

After closing the gripper and lifting, the arm goes back to the height it was at
before grabbing and looks down with its camera:
  - the cube still on the floor looks the same size as before  -> the grab missed:
    open, line up again and grab again on the spot (up to GRAB_TRIES times)
  - the cube is much bigger (it is in the gripper, close to the camera) or not on the
    floor any more                                              -> held: carry it to the basket

Usage on the robot:  python3 ~/patch_holdcheck.py   (patches ~/fetch_blue.py)
"""
import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'def holding(' in s:
    print('already patched')
    sys.exit(0)

HELPERS = '''def cube_seen(frames=12):
    """Where the cube is in the next few pictures and how big it looks (median), or None."""
    tr, seq, seen = Tracker(), latest['seq'], []
    for _ in range(frames):
        seq, strict, loose, _ = next_frame(seq)
        pos, fresh = tr.update(strict, loose)
        if fresh and pos:
            seen.append((pos[0], pos[1], tr.area))
    if len(seen) < 3:
        return None
    med = lambda i: sorted(q[i] for q in seen)[len(seen) // 2]
    return med(0), med(1), med(2)


def holding(base, reach, ref):
    """Back at the height the arm lined up at, look down: is the cube still on the floor?"""
    say('Checking the cube is in the gripper')
    ik_move(reach, HOVER_Z, base, 900)
    pause(1.2)
    now = cube_seen()
    if now is None:
        print('  no cube on the floor any more: it is in the gripper')
        return True
    cx, cy, area = now
    if ref is None:
        on_floor = abs(cx - CENTRE_X) < 90 and abs(cy - CENTRE_Y) < 90
    else:
        print('  cube size now %d pixels, before the grab %d' % (area, ref[2]))
        on_floor = area < ref[2] * HELD_GROWTH          # same size as before: still lying on the floor
    if on_floor:
        print('  the cube is still on the floor at (%d, %d)' % (cx, cy))
        return False
    print('  cube is close to the camera: it is in the gripper')
    return True


def pick_up(base, reach, pitch, wrist):
    """Grab, check, and grab again on the spot until the cube is held. Returns True when held."""
    ref = cube_seen()
    for g in range(1, GRAB_TRIES + 1):
        if g > 1:
            say('Missed the cube - opening and grabbing again (%d of %d)' % (g, GRAB_TRIES))
            servos(500, ((1, GRIP_OPEN),))
            pause(0.6)
            res, data = track(base, reach)
            if res != 'ok':
                say('Lost the cube after the missed grab')
                return False
            base, reach, pitch = data
            if WRIST_ALIGN:
                wrist = align_wrist()
                if wrist != 500:
                    res, data = track(base, reach)
                    if res == 'ok':
                        base, reach, pitch = data
            ref = cube_seen()
        if not grab(base, reach, pitch, wrist):
            return False
        if holding(base, reach, ref):
            carry()
            return True
    say('Could not get hold of the cube after %d grabs' % GRAB_TRIES)
    servos(500, ((1, GRIP_OPEN),))
    pause(0.6)
    return False


'''

R = [
    # settings
    ("SCAN = [(500, 0.15),",
     "GRAB_TRIES = 3             # grabs on the spot before starting the whole attempt again\n"
     "HELD_GROWTH = 1.8          # cube in the gripper looks at least this much bigger than on the floor\n"
     "SCAN = [(500, 0.15),"),
    # remember how big the cube looks
    ("        self.pos, self.missed, self.fresh, self.angle = None, 0, 0, 0.0\n",
     "        self.pos, self.missed, self.fresh, self.angle, self.area = None, 0, 0, 0.0, 0\n"),
    ("            self.angle = cand[4]\n",
     "            self.angle = cand[4]\n            self.area = cand[2]\n"),
    # grab() stops after the lift; carrying is its own step, only once the cube is held
    ("""    ik_move(reach + offset, GRAB_Z + LIFT_FIRST, base, 800, extra=((2, wrist),))   # lift straight up first
    pause(1.0)
    servos(1500, ((1, GRIP_CLOSE + GRIP_EXTRA), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))   # carry
    pause(1.6)
    return True
""",
     """    ik_move(reach + offset, GRAB_Z + LIFT_FIRST, base, 800, extra=((2, wrist),))   # lift straight up first
    pause(1.0)
    return True


def carry():
    servos(1500, ((1, GRIP_CLOSE + GRIP_EXTRA), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))
    pause(1.6)
"""),
    ("def place_left():\n", HELPERS + "def place_left():\n"),
    # main loop: only place when held
    ("""        if not grab(base, reach, pitch, wrist):
            continue
        place_left()
""",
     """        if not pick_up(base, reach, pitch, wrist):
            continue
        place_left()
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
