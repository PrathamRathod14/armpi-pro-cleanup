"""frontarm: if the front camera sees no cube, look with the arm camera too before giving up.

A cube the front camera misses (for example a blue it is not tuned for) is still picked
when it lies within the arm's reach. Needs patch_red.py first.

Usage on the robot:  python3 ~/patch_armcheck.py   (patches ~/fetch_blue.py)
"""
import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'checking with the arm camera' in s:
    print('already patched')
    sys.exit(0)
old = """            if not det:
                if placed:
                    break
                say('The front camera does not see a cube - place one in front of the robot')
                time.sleep(3)
                continue
            order_scan(det[0])
"""
new = """            if not det:
                say('Front camera sees no cube - checking with the arm camera')
                if not arm_find():
                    if placed:
                        break
                    say('The front camera does not see a cube - place one in front of the robot')
                    time.sleep(3)
                    continue
            else:
                order_scan(det[0])
"""
if old not in s:
    print('nothing changed, could not find the frontarm block (apply patch_red.py first)')
    sys.exit(0)
open(path, 'w').write(s.replace(old, new, 1))
print('patched')
