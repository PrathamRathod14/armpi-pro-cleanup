import os, sys
files = sys.argv[1:] or [os.path.expanduser('~/grasp_ik.py'), os.path.expanduser('~/grasp_black.py')]
R = []
R.append(("DEADBAND = 10                   # pixels: close enough to the target point\n",
          "DEADBAND = 10                   # pixels: close enough to the target point\n"
          "ON_TARGET = 20                  # pixels: must be this close to the cross before grabbing\n"
          "MAX_REACH = 0.25                # metres: farthest the gripper can still grab at table height\n"
          "JUMP = 140                      # pixels: a bigger jump between frames means a different object\n"))
R.append(("""    cube = None
    for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
        a = cv2.contourArea(c)
        if a >= MIN_AREA and (cube is None or a > cube[2]):
            m = cv2.moments(c)
            cx, cy = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
            cube = (cx, cy, a, cv2.boundingRect(c))
    with lock:
        latest['seq'] += 1
        latest['cube'] = cube
""", """    blobs = []
    for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
        a = cv2.contourArea(c)
        if a >= MIN_AREA:
            m = cv2.moments(c)
            cx, cy = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
            blobs.append((cx, cy, a, cv2.boundingRect(c)))
    blobs.sort(key=lambda b: -b[2])
    cube = blobs[0] if blobs else None
    with lock:
        latest['seq'] += 1
        latest['cube'] = cube
        latest['blobs'] = blobs
"""))
R.append(("""    last_seq, stable, t_lost, t0 = cube_now()[0], 0, time.time(), time.time()
    shown = 0.0""", """    last_seq, stable, t_lost, t0 = cube_now()[0], 0, time.time(), time.time()
    shown, prev, at_limit = 0.0, None, 0"""))
R.append(("""        last_seq = seq
        if cube is None:
            stable = 0""", """        last_seq = seq
        if prev is not None:                 # follow the same object, not whatever is biggest
            with lock:
                blobs = list(latest.get('blobs', []))
            near = [b for b in blobs if (b[0] - prev[0]) ** 2 + (b[1] - prev[1]) ** 2 < JUMP ** 2]
            cube = min(near, key=lambda b: (b[0] - prev[0]) ** 2 + (b[1] - prev[1]) ** 2) if near else None
        if cube is None:
            stable = 0"""))
R.append(("""        cx, cy = cube[0], cube[1]
        dx = X_GAIN""", """        cx, cy = cube[0], cube[1]
        prev = (cx, cy)
        dx = X_GAIN"""))
R.append(("        y_dis = clamp(y_dis + dy, 0.12, 0.28)\n", "        y_dis = clamp(y_dis + dy, 0.12, MAX_REACH)\n"))
R.append(("""        if abs(dx) < 2 and abs(dy) < 0.003 and t:
            stable += 1""", """        if y_dis >= MAX_REACH and cy < CENTRE_Y - ON_TARGET:
            at_limit += 1
            if at_limit >= 15:
                say('Object is beyond reach - move it closer (12-25 cm ahead)')
                return None
        else:
            at_limit = 0
        if abs(cx - CENTRE_X) < ON_TARGET and abs(cy - CENTRE_Y) < ON_TARGET and t:
            stable += 1"""))
for path in files:
    try:
        s = open(path).read()
    except IOError:
        print(path, ': not found, skipped')
        continue
    if 'ON_TARGET' in s:
        print(path, ': already patched')
        continue
    missing = [old[:40] for old, new in R if old not in s]
    if missing:
        print(path, ': nothing changed, could not find:', missing)
        continue
    for old, new in R:
        s = s.replace(old, new, 1)
    open(path, 'w').write(s)
    print(path, ': patched')
