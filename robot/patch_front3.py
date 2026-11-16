import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'class FrontTopic(' in s:
    print('already patched')
    sys.exit(0)

TOPIC_CLASS = '''class FrontTopic(object):
    """Reads the front camera's detections from front_cam.py (topic /front_cam/target)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.det, self.t, self.count = None, 0.0, 0
        rospy.Subscriber('/front_cam/target', String, self.on_msg, queue_size=5)

    def on_msg(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        det = (d['cx'], d['cy'], d['w'], d['h'], d['pixels']) if d.get('found') else None
        with self.lock:
            self.det, self.t = det, time.time()
            self.count += 1

    def latest(self, last_count, timeout=1.0):
        end = time.time() + timeout
        while time.time() < end and not rospy.is_shutdown():
            with self.lock:
                if self.count != last_count:
                    return self.count, self.det
            time.sleep(0.01)
        return last_count, None

    def stop(self):
        pass


front = None
'''

R = [
    ("\nfront = None\n", "\n" + TOPIC_CLASS),
    ("        front = FrontCam(FRONT_PORT)\n"
     "        front.start()\n"
     "        rospy.on_shutdown(front.stop)\n"
     "        print('Starting the front camera program on the K210 module...')\n",
     "        front = FrontTopic()\n"
     "        print('Waiting for the front camera (front_cam.py must be running in another window)...')\n"),
    ("            print('No results from the front camera on %s - is the module plugged in with data?' % FRONT_PORT)\n",
     "            print('No results from the front camera - start it first in another window: python3 ~/front_cam.py')\n"),
]
missing = [old[:60] for old, new in R if old not in s]
if missing:
    print('nothing changed, could not find:', missing)
    sys.exit(0)
for old, new in R:
    s = s.replace(old, new, 1)
open(path, 'w').write(s)
print('patched')
