#!/usr/bin/env python3
# Keyboard teleop for the ArmPi Pro: chassis + arm + gripper + home pose.
import sys, os, json, select, termios, tty, signal, time
import rospy, rostopic, roslib.message

CHASSIS_TOPIC = '/chassis_control/set_velocity'
SERVO_TOPIC = '/servo_controllers/port_id_1/multi_id_pos_dur'
STATE_TOPIC = '/servo_controllers/port_id_1/servo_states'
HOME_FILE = os.path.expanduser('~/arm_home.json')

SPEED = 80.0      # chassis speed, mm/s
TURN = 0.3        # chassis rotation rate
TIMEOUT = 0.6     # chassis stops if no key arrives for this long (seconds)
STEP = 8          # servo units per key repeat (1000 units = about 240 degrees)
ARM_PERIOD = 0.05 # minimum time between arm commands (seconds)
HOME_TIME = 2000  # time to move back to home (milliseconds)
STOP = (0.0, 0.0, 0.0)

# chassis keys: (velocity, direction in degrees, angular)
DRIVE = {
    'w': (SPEED, 90.0, 0.0),    # forward
    's': (SPEED, 270.0, 0.0),   # backward
    'a': (SPEED, 180.0, 0.0),   # slide left
    'd': (SPEED, 0.0, 0.0),     # slide right
    'q': (0.0, 90.0, TURN),     # rotate one way
    'e': (0.0, 90.0, -TURN),    # rotate the other way
}

# arm keys: (servo id, direction)
ARM = {
    'j': (6, +1), 'l': (6, -1),   # base rotate
    'i': (5, +1), 'k': (5, -1),   # shoulder
    'u': (4, +1), 'o': (4, -1),   # elbow
    'y': (3, +1), 'h': (3, -1),   # wrist up/down
    'n': (2, +1), 'm': (2, -1),   # wrist rotate
    'r': (1, +1), 'f': (1, -1),   # gripper
}

HELP = """
Chassis:  w/s forward/back   a/d slide left/right   q/e rotate   (release = stop)
Arm:      j/l base   i/k shoulder   u/o elbow   y/h wrist   n/m wrist rotate
Gripper:  r/f
Home:     g go to home pose   t save current pose as home
Other:    p print servo positions   space stop chassis   x quit
"""

def read_positions():
    """Read current servo positions so the arm does not jump on the first key."""
    cls = rostopic.get_topic_class(STATE_TOPIC)[0]
    if cls is None:
        return {}
    try:
        msg = rospy.wait_for_message(STATE_TOPIC, cls, timeout=3.0)
    except rospy.ROSException:
        return {}
    for slot in msg.__slots__:
        val = getattr(msg, slot)
        if isinstance(val, (list, tuple)) and val and hasattr(val[0], 'id') and hasattr(val[0], 'position'):
            return dict((int(s.id), int(s.position)) for s in val)
    return {}

def load_home():
    try:
        with open(HOME_FILE) as f:
            return dict((int(k), int(v)) for k, v in json.load(f).items())
    except (IOError, OSError, ValueError):
        return {}

def save_home(pos):
    with open(HOME_FILE, 'w') as f:
        json.dump(dict((str(k), v) for k, v in pos.items()), f)

def main():
    rospy.init_node('teleop_keys', anonymous=True)
    chassis_cls = rostopic.get_topic_class(CHASSIS_TOPIC)[0]
    servo_cls = rostopic.get_topic_class(SERVO_TOPIC)[0]
    if chassis_cls is None or servo_cls is None:
        print('Cannot find the chassis or servo topic - is the robot stack running?')
        return
    item_cls = roslib.message.get_message_class(servo_cls._slot_types[0].replace('[]', ''))
    chassis_pub = rospy.Publisher(CHASSIS_TOPIC, chassis_cls, queue_size=1)
    servo_pub = rospy.Publisher(SERVO_TOPIC, servo_cls, queue_size=1)
    rospy.on_shutdown(lambda: chassis_pub.publish(*STOP))
    signal.signal(signal.SIGHUP, lambda *a: rospy.signal_shutdown('hangup'))

    pos = read_positions()
    if pos:
        print('Servo positions: %s' % sorted(pos.items()))
    else:
        print('WARNING: could not read servo positions. Each joint will start from 500')
        print('on its first key press and may make one larger move. Keep clear.')
    home = load_home()
    if home:
        print('Saved home pose: %s' % sorted(home.items()))
    else:
        print('No home pose saved yet. Put the arm where you want it and press t.')

    print(HELP)
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    current, last_key, last_arm = STOP, 0.0, 0.0
    try:
        while not rospy.is_shutdown():
            ready = select.select([sys.stdin], [], [], 0.05)[0]
            cmd = current
            if ready:
                k = sys.stdin.read(1).lower()
                now = time.time()
                if k == 'x':
                    break
                if k == 'p':
                    print('Servo positions: %s' % sorted(pos.items()))
                elif k == 't':
                    if pos:
                        home = dict(pos)
                        save_home(home)
                        print('Home pose saved: %s' % sorted(home.items()))
                    else:
                        print('Nothing to save yet - positions unknown.')
                elif k == 'g':
                    cmd = STOP
                    if home and now >= last_arm:
                        servo_pub.publish(servo_cls([item_cls(sid, p, HOME_TIME) for sid, p in sorted(home.items())]))
                        pos.update(home)
                        last_arm = now + HOME_TIME / 1000.0
                        print('Going home...')
                    elif not home:
                        print('No home pose saved. Press t to save one.')
                elif k in ARM:
                    cmd = STOP
                    if now - last_arm >= ARM_PERIOD:
                        sid, sign = ARM[k]
                        first = sid not in pos
                        target = max(0, min(1000, pos.get(sid, 500) + sign * STEP))
                        pos[sid] = target
                        duration = 1500 if first else int(ARM_PERIOD * 1000) + 20
                        servo_pub.publish(servo_cls([item_cls(sid, target, duration)]))
                        last_arm = now + (1.5 if first else 0.0)
                else:
                    cmd = DRIVE.get(k, STOP)
                    last_key = now
            elif time.time() - last_key > TIMEOUT:
                cmd = STOP
            if cmd != current:
                chassis_pub.publish(*cmd)
                current = cmd
    finally:
        chassis_pub.publish(*STOP)
        time.sleep(0.2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)

if __name__ == '__main__':
    main()
