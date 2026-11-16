"""Pick red cubes as well as blue ones, one after another, until none is left.

Patches ~/fetch_blue.py (arm camera + main loop) and ~/front_cam.py (front camera).
Set PICK_RED = False at the top of fetch_blue.py / front_cam.py to go back to blue only.

Usage on the robot:  python3 ~/patch_red.py
"""
import os, sys
home = os.path.expanduser('~')


def patch(path, marker, R):
    s = open(path).read()
    if marker in s:
        print(os.path.basename(path), ': already patched')
        return
    missing = [old[:60] for old, new in R if old not in s]
    if missing:
        print(os.path.basename(path), ': nothing changed, could not find:', missing)
        return
    for old, new in R:
        s = s.replace(old, new, 1)
    open(path, 'w').write(s)
    print(os.path.basename(path), ': patched')


# ---------------- fetch_blue.py ----------------
patch(os.path.join(home, 'fetch_blue.py'), 'PICK_RED', [
    # colour ranges: red sits at both ends of the hue scale (0-8 and 170-180)
    ("LOOSE = ((90, 70, 60), (135, 255, 255))      # to KEEP tracking it: also shaded faces and highlights\n",
     "LOOSE = ((90, 70, 60), (135, 255, 255))      # to KEEP tracking it: also shaded faces and highlights\n"
     "PICK_RED = True            # also pick red cubes\n"
     "STRICT_RED = (((0, 120, 80), (8, 255, 255)), ((170, 120, 80), (180, 255, 255)))\n"
     "LOOSE_RED = (((0, 90, 60), (9, 255, 255)), ((165, 90, 60), (180, 255, 255)))\n"),
    ("def blobs_in(hsv, rng, min_area, shape_check):\n    mask = cv2.inRange(hsv, np.array(rng[0]), np.array(rng[1]))\n",
     "def blobs_in(hsv, rngs, min_area, shape_check):\n"
     "    mask = None\n"
     "    for rng in rngs:                                  # one or more colour ranges, joined\n"
     "        m = cv2.inRange(hsv, np.array(rng[0]), np.array(rng[1]))\n"
     "        mask = m if mask is None else cv2.bitwise_or(mask, m)\n"),
    ("    return blobs_in(hsv, STRICT, MIN_AREA, True), blobs_in(hsv, LOOSE, MIN_AREA // 2, False)\n",
     "    strict = (STRICT,) + (STRICT_RED if PICK_RED else ())\n"
     "    loose = (LOOSE,) + (LOOSE_RED if PICK_RED else ())\n"
     "    return blobs_in(hsv, strict, MIN_AREA, True), blobs_in(hsv, loose, MIN_AREA // 2, False)\n"),
    # front camera program started from fetch_blue.py
    ("                                  th=str(tuple(FRONT_BLUE)), px=FRONT_MIN_PIXELS).encode()\n",
     "                                  th=', '.join(str(tuple(t)) for t in [FRONT_BLUE] + ([FRONT_RED] if PICK_RED else [])),\n"
     "                                  px=FRONT_MIN_PIXELS).encode()\n"),
    ("FRONT_MIN_PIXELS = 60 ",
     "FRONT_RED = (20, 80, 30, 80, 0, 60)   # LAB colour range of the red cube\nFRONT_MIN_PIXELS = 60 "),
    ("    say('Front camera: looking for the blue cube')\n",
     "    say('Front camera: looking for a blue or red cube' if PICK_RED else 'Front camera: looking for the blue cube')\n"),
    # main loop: after a cube is in the basket, go on with the next one
    ("    for attempt in range(1, MAX_TRIES + 1):\n",
     "    placed, attempt = 0, 0\n    while attempt < MAX_TRIES:\n        attempt += 1\n"),
    ("            if not det:\n                say('The front camera does not see the blue cube - place it in front of the robot')\n",
     "            if not det:\n                if placed:\n                    break\n"
     "                say('The front camera does not see a cube - place one in front of the robot')\n"),
    ("                say('The front camera sees no blue cube')\n                return\n",
     "                if placed:\n                    break\n                say('The front camera sees no cube')\n                return\n"),
    ("                say('No blue cube found anywhere around the robot')\n                return\n",
     "                if placed:\n                    break\n                say('No cube found anywhere around the robot')\n                return\n"),
    ("        above = line_up()\n        if not above:\n            continue\n",
     "        above = line_up()\n        if not above:\n            if placed and MODE in ('arm', 'vlm'):\n                break\n            continue\n"),
    ("        arm_look()\n        say('Done: blue cube placed in the basket')\n        return\n",
     "        placed += 1\n        attempt = 0\n"
     "        say('Cube %d is in the basket - looking for the next one' % placed)\n        continue\n"),
    ("    stop()\n    arm_look()\n    say('Gave up after %d attempts' % MAX_TRIES)\n",
     "    stop()\n    arm_look()\n"
     "    if placed:\n        say('Finished: %d cube(s) in the basket, no more cubes found' % placed)\n"
     "    else:\n        say('Gave up after %d attempts' % MAX_TRIES)\n"),
])

# ---------------- front_cam.py ----------------
patch(os.path.join(home, 'front_cam.py'), 'PICK_RED', [
    ("BLUE = (20, 85, -25, 15, -70, -15)   # LAB range of the blue cube: L min/max, A min/max, B min/max\n",
     "BLUE = (20, 85, -25, 15, -70, -15)   # LAB range of the blue cube: L min/max, A min/max, B min/max\n"
     "RED = (20, 80, 30, 80, 0, 60)        # LAB range of the red cube (A positive = red)\n"
     "PICK_RED = True                      # also report red cubes\n"),
    ("th=str(tuple(BLUE)), px=MIN_PIXELS,",
     "th=', '.join(str(tuple(t)) for t in [BLUE] + ([RED] if PICK_RED else [])), px=MIN_PIXELS,"),
])
