import os, sys
path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fetch_blue.py')
s = open(path).read()
if 'wait for the >>> prompt' in s:
    print('already patched')
    sys.exit(0)
old = """        self.ser = s = serial.Serial(self.port, 115200, timeout=0.1)
        time.sleep(2.5)                     # opening the port can restart the module: let it boot
        s.reset_input_buffer()
        s.write(b'\\r\\x03\\x03')              # stop whatever program it is running
        time.sleep(0.5)
        s.write(b'\\r\\x01')                  # raw REPL
"""
new = """        s = serial.Serial()
        s.port, s.baudrate, s.timeout = self.port, 115200, 0.1
        s.dtr = False                       # do not hold the module's reset/boot pins
        s.rts = False
        s.open()
        self.ser = s
        print('  waiting for the module to start (about 6 s)...')
        time.sleep(6)                       # let ACEBOTT's program finish starting
        s.reset_input_buffer()
        got, end = b'', time.time() + 8     # stop it: Ctrl-C until we see the >>> prompt
        while b'>>>' not in got and time.time() < end:
            s.write(b'\\x03')
            time.sleep(0.3)
            got += s.read(2000)
        if b'>>>' not in got:
            raise IOError('the module did not stop its own program (no >>> prompt)')
        time.sleep(0.5)
        s.read(4000)                        # wait for the >>> prompt, then drop the rest
        s.write(b'\\r\\x01')                  # raw REPL
"""
if old not in s:
    print('nothing changed: front camera start-up code not found')
    sys.exit(0)
s = s.replace(old, new, 1)
s = s.replace("while front.count == 0 and time.time() - t_wait < 12 and not rospy.is_shutdown():",
              "while front.count == 0 and time.time() - t_wait < 25 and not rospy.is_shutdown():")
open(path, 'w').write(s)
print('patched')
