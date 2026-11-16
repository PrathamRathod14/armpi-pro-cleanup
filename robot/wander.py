#!/usr/bin/env python3
# Wander: the robot drives around on its own - forward, back, sideways, turning - with gentle starts.
#   python3 ~/wander.py          for 60 seconds
#   python3 ~/wander.py 120      for 120 seconds
# Floor only, in open space: it has no obstacle sensor. Ctrl+C stops it at once.
import sys, random, time
import rospy
import rostopic
from chassis_control.msg import SetVelocity

SPEED = 60.0          # mm/s
TURN = 0.3            # turn rate
MIN_BATTERY = 7200    # mV: below this it will not drive
MOVES = [             # name, direction (90 = forward), speed factor, turn
    ('forward', 90, 1, 0), ('back', 270, 1, 0), ('sideways left', 180, 1, 0),
    ('sideways right', 0, 1, 0), ('turn left', 90, 0, TURN), ('turn right', 90, 0, -TURN),
]

rospy.init_node('wander')
pub = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
stop = lambda: pub.publish(0.0, 90.0, 0.0)
rospy.on_shutdown(stop)
rospy.sleep(0.5)

try:
    cls = rostopic.get_topic_class('/sensor/battery')[0]
    mv = int(rospy.wait_for_message('/sensor/battery', cls, timeout=3.0).data)
    print('Battery %.2f V' % (mv / 1000.0))
    if mv < MIN_BATTERY:
        sys.exit('Battery too low for driving - charge it first.')
except rospy.ROSException:
    print('Battery reading not available - driving anyway, watch it.')

end = time.time() + (float(sys.argv[1]) if len(sys.argv) > 1 else 60)
last = None
while not rospy.is_shutdown() and time.time() < end:
    name, direction, k, turn = random.choice([m for m in MOVES if m[0] != last])
    last, secs = name, random.uniform(1.0, 2.5)
    print('%-15s %.1f s' % (name, secs))
    t0 = time.time()
    while not rospy.is_shutdown() and time.time() - t0 < secs:
        ramp = min(1.0, (time.time() - t0) / 0.8, (secs - (time.time() - t0)) / 0.5)   # gentle start and stop
        pub.publish(SPEED * k * max(ramp, 0), float(direction), turn * max(ramp, 0))
        rospy.sleep(0.1)
    stop()
    rospy.sleep(0.6)
stop()
print('Done wandering.')
