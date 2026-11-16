"""Let the robot talk ("Got it! I picked up the cube.") or, without a speaker, beep.

It speaks with espeak / espeak-ng / spd-say if one is installed and a speaker is connected
to the Pi (3.5 mm jack or USB). Otherwise it uses the robot's buzzer:
  2 short beeps = success, 1 long beep = problem, 1 tiny beep = starting something.
Speech runs in the background, so the arm never waits for it.

Settings at the top of fetch_blue.py after patching:
  VOICE = 'auto'   'auto' (talk if possible, else beep), 'beep' (buzzer only), 'off'

Usage on the robot:  python3 ~/patch_voice.py   (patches ~/fetch_blue.py)
"""
import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'def talk(' in s:
    print('already patched')
    sys.exit(0)

VOICE_CODE = '''VOICE = 'auto'             # 'auto' = talk if a speaker and espeak are there, else beep; 'beep'; 'off'
VOICE_SPEED = 150          # words per minute
OK, BAD, INFO = (0.08, 0.08), (0.45,), (0.04,)     # buzzer patterns (seconds of each beep)
SAYINGS = [                # (start of a status message, what the robot says, beeps without a speaker)
    ('Searching',               'Searching for cubes.', INFO),
    ('Grabbing',                "I'm trying to grab it.", INFO),
    ('Missed the cube',         'Oops, I missed. Trying again.', BAD),
    ('Could not get hold',      'I could not get hold of it.', BAD),
    ('Lost the cube',           'I lost the cube.', BAD),
    ('The cube is still there', 'Hmm, the cube is still there. Trying again.', BAD),
    ('Placing in the basket',   'Putting it in my basket.', INFO),
    ('Done',                    'Done! The cube is in my basket.', OK),
    ('Cube too far',            'That cube is too far for my arm. Please move it closer.', BAD),
    ('Cube too close',          'That cube is too close. Please move it back a little.', BAD),
    ('Cube a little too far',   'Moving a little closer.', INFO),
    ('Finished',                'All done! Every cube is in my basket.', OK),
    ('Gave up',                 'Sorry, I could not do it.', BAD),
    ('The front camera does not see', "I can't see any cube.", BAD),
    ('The front camera sees no', "I can't see any cube.", BAD),
    ('No blue cube found',      "I can't find any cube.", BAD),
    ('No cube found',           "I can't find any cube.", BAD),
]
_voice_q, _voice_cmd, _voice_last = None, None, {}


def _voice_worker():
    import subprocess
    while True:
        text, beeps = _voice_q.get()
        try:
            if _voice_cmd:
                subprocess.call(_voice_cmd + [text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            elif buzzer_pub is not None:
                for b in beeps:
                    buzzer_pub.publish(b)
                    time.sleep(b + 0.12)
        except Exception as e:
            print('  (voice: %s)' % e)


def talk(text, beeps=INFO):
    """Say something in the background (or beep). The same sentence is not repeated within 15 s."""
    global _voice_q, _voice_cmd
    if VOICE == 'off' or time.time() - _voice_last.get(text, 0) < 15:
        return
    _voice_last[text] = time.time()
    if _voice_q is None:
        import queue, shutil
        _voice_q = queue.Queue()
        if VOICE == 'auto':
            for prog in ('espeak-ng', 'espeak', 'spd-say'):
                if shutil.which(prog):
                    _voice_cmd = [prog, '-s', str(VOICE_SPEED)] if prog != 'spd-say' else [prog, '-w']
                    break
        print('  (voice: %s)' % ('talking with ' + _voice_cmd[0] if _voice_cmd else 'buzzer beeps'))
        threading.Thread(target=_voice_worker, daemon=True).start()
    while _voice_q.qsize() >= 2:               # never fall behind: drop the oldest waiting sentence
        try:
            _voice_q.get_nowait()
        except Exception:
            break
    print('  robot says: "%s"' % text)
    _voice_q.put((text, beeps))


'''

R = [
    ("def say(text):\n    global status\n    status = text\n    print(text)\n",
     VOICE_CODE +
     "def say(text):\n    global status\n    status = text\n    print(text)\n"
     "    for start, words, beeps in SAYINGS:\n"
     "        if text.startswith(start):\n"
     "            talk(words, beeps)\n"
     "            break\n"
     "    if text.startswith('Cube ') and 'in the basket' in text:\n"
     "        talk('Done! Looking for the next one.', OK)\n"),
    ("    if not ARM_ONLY:\n        mv = battery_mv()\n",
     "    talk('Hello! I am looking for cubes to put in my basket.', INFO)\n"
     "    if not ARM_ONLY:\n        mv = battery_mv()\n"),
]
OPTIONAL = [   # only there after patch_holdcheck.py
    ("def carry():\n", "def carry():\n    talk('Got it! I picked up the cube.', OK)\n"),
]
missing = [old[:60] for old, new in R if old not in s]
if missing:
    print('nothing changed, could not find:', missing)
    sys.exit(0)
for old, new in R + [r for r in OPTIONAL if r[0] in s]:
    s = s.replace(old, new, 1)
open(path, 'w').write(s)
print('patched')
