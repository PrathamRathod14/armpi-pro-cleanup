import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'Placing in the basket' in s:
    print('already patched')
    sys.exit(0)
old = """    say('Placing on the left')
    d = poses['drop']
    servos(1200, ((6, d[6]),))
    pause(1.3)
    servos(1500, ((3, d[3]), (4, d[4]), (5, d[5])))
    pause(1.8)
    servos(500, ((1, GRIP_OPEN),))
    pause(0.8)
    servos(1200, ((3, 80), (4, 825), (5, 625)))
    pause(1.3)
"""
new = """    say('Placing in the basket')
    d = poses['drop']                                   # taught DROP pose: cube held above the basket
    servos(1500, ((6, d[6]),))                          # turn first, arm still raised in the carry pose
    pause(1.6)
    servos(2200, ((2, d.get(2, 500)), (3, d[3]), (4, d[4]), (5, d[5])))   # slowly over the basket
    pause(2.4)
    servos(500, ((1, GRIP_OPEN),))                      # let go: the cube drops in
    pause(0.8)
    servos(1800, ((3, 80), (4, 825), (5, 625)))         # back up to the carry pose before turning
    pause(2.0)
    servos(1500, ((2, 500), (6, 500)))
    pause(1.6)
"""
if old not in s:
    print('nothing changed: placing routine not found')
    sys.exit(0)
s = s.replace(old, new, 1)
s = s.replace("say('Done: blue cube placed on the left')", "say('Done: blue cube placed in the basket')")
open(path, 'w').write(s)
print('patched')
