import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'def align_wrist(' in s:
    print('already patched')
    sys.exit(0)
R = [
    # settings
    ("SCAN = [(500, 0.15)", "WRIST_ALIGN = True         # turn the gripper to match the cube's sides before grabbing\n"
     "WRIST_SIGN = 1             # reversed automatically if turning the wrist makes it worse\n"
     "WRIST_UNITS_PER_DEG = 1000 / 240.0\n"
     "GRIP_EXTRA = 25            # second, firmer squeeze after the first close\n"
     "LIFT_FIRST = 0.04          # lift straight up this far (metres) before swinging away\n"
     "SCAN = [(500, 0.15)"),
    # measure each patch's angle
    ("        m = cv2.moments(c)\n        out.append((m['m10'] / m['m00'], m['m01'] / m['m00'], a, (x, y, bw, bh)))",
     "        m = cv2.moments(c)\n"
     "        ang = cv2.minAreaRect(c)[2] % 90.0\n"
     "        ang = ang - 90.0 if ang > 45.0 else ang          # -45..45 degrees: how far the sides are turned\n"
     "        out.append((m['m10'] / m['m00'], m['m01'] / m['m00'], a, (x, y, bw, bh), ang))"),
    ("        self.pos, self.missed, self.fresh = None, 0, 0\n",
     "        self.pos, self.missed, self.fresh, self.angle = None, 0, 0, 0.0\n"),
    ("            self.missed = 0\n            self.fresh += 1\n",
     "            self.missed = 0\n            self.fresh += 1\n            self.angle = cand[4]\n"),
    # firmer, gentler grab
    ("    servos(500, ((1, GRIP_CLOSE),))\n    pause(0.8)\n"
     "    servos(1500, ((1, GRIP_CLOSE), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))   # lift\n",
     "    servos(500, ((1, GRIP_CLOSE),))\n    pause(0.8)\n"
     "    servos(300, ((1, GRIP_CLOSE + GRIP_EXTRA),))         # second squeeze to seat the cube\n    pause(0.5)\n"
     "    ik_move(reach + offset, GRAB_Z + LIFT_FIRST, base, 800, extra=((2, wrist),))   # lift straight up first\n    pause(1.0)\n"
     "    servos(1500, ((1, GRIP_CLOSE + GRIP_EXTRA), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))   # carry\n"),
    # wrist alignment step, inserted before grabbing
    ("# ---------------- 3. grab, place, check ----------------\n",
     "def measure_angle(frames=8):\n"
     "    tr, seq, angles = Tracker(), latest['seq'], []\n"
     "    for _ in range(frames * 2):\n"
     "        seq, strict, loose, _ = next_frame(seq)\n"
     "        pos, fresh = tr.update(strict, loose)\n"
     "        if fresh:\n"
     "            angles.append(tr.angle)\n"
     "        if len(angles) >= frames:\n"
     "            break\n"
     "    return sorted(angles)[len(angles) // 2] if angles else None\n\n\n"
     "def align_wrist():\n"
     "    \"\"\"Turn the gripper so its fingers close on two flat sides of the cube, not on its corners.\"\"\"\n"
     "    global WRIST_SIGN\n"
     "    ang = measure_angle()\n"
     "    if ang is None or abs(ang) < 4:\n"
     "        return 500\n"
     "    say('Turning the gripper %.0f degrees to match the cube' % ang)\n"
     "    wrist = int(clamp(500 + WRIST_SIGN * ang * WRIST_UNITS_PER_DEG, 300, 700))\n"
     "    servos(500, ((2, wrist),))\n"
     "    pause(0.8)\n"
     "    after = measure_angle()\n"
     "    if after is not None and abs(after) > abs(ang):            # turned the wrong way\n"
     "        WRIST_SIGN = -WRIST_SIGN\n"
     "        wrist = int(clamp(500 + WRIST_SIGN * ang * WRIST_UNITS_PER_DEG, 300, 700))\n"
     "        servos(500, ((2, wrist),))\n"
     "        pause(0.8)\n"
     "        after = measure_angle()\n"
     "    print('  cube sides now at %s degrees (wrist %d)' % ('?' if after is None else '%.0f' % after, wrist))\n"
     "    return wrist\n\n\n"
     "# ---------------- 3. grab, place, check ----------------\n"),
    ("def grab(base, reach, pitch):\n", "def grab(base, reach, pitch, wrist=500):\n"),
    ("    if not ik_move(reach + offset, GRAB_Z, base, 1000):\n",
     "    if not ik_move(reach + offset, GRAB_Z, base, 1000, extra=((2, wrist),)):\n"),
    # use it in the main loop: align, re-centre, grab
    ("        base, reach, pitch = above\n        if not grab(base, reach, pitch):\n",
     "        base, reach, pitch = above\n"
     "        wrist = 500\n"
     "        if WRIST_ALIGN:\n"
     "            wrist = align_wrist()\n"
     "            if wrist != 500:                                  # re-centre after turning the wrist\n"
     "                res, data = track(base, reach)\n"
     "                if res == 'ok':\n"
     "                    base, reach, pitch = data\n"
     "        if not grab(base, reach, pitch, wrist):\n"),
]
missing = [old[:60] for old, new in R if old not in s]
if missing:
    print('nothing changed, could not find:', missing)
    sys.exit(0)
for old, new in R:
    s = s.replace(old, new, 1)
open(path, 'w').write(s)
print('patched')
