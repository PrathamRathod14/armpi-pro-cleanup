"""Phase 2: check the laptop can talk to the robot through its ROS bridge (port 9090).

    python phase2_test.py --robot 10.238.54.20 --seconds 90

1. Connects to ws://ROBOT:9090 and lists the robot's topics.
2. Publishes test messages on /vlm/target (std_msgs/String, JSON) twice a second.
3. Listens to /vlm/target itself, to measure how long a message takes to go
   laptop -> robot -> laptop.
On the robot, `rostopic echo /vlm/target` should show the same messages.
"""
import argparse
import json
import statistics
import time

import roslibpy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', default='10.238.54.20')
    ap.add_argument('--seconds', type=float, default=90)
    args = ap.parse_args()

    client = roslibpy.Ros(host=args.robot, port=9090)
    t0 = time.time()
    client.run(timeout=10)
    print('connected to ws://%s:9090 in %.2f s' % (args.robot, time.time() - t0))

    topics = client.get_topics()
    print('robot has %d topics, for example:' % len(topics))
    for name in ('/usb_cam/image_raw', '/chassis_control/set_velocity',
                 '/servo_controllers/port_id_1/multi_id_pos_dur', '/cleanup/image'):
        print('  %-48s %s' % (name, 'yes' if name in topics else 'no'))

    delays = []

    def on_msg(msg):
        try:
            data = json.loads(msg['data'])
            delays.append((time.time() - data['sent_time']) * 1000.0)
        except (ValueError, KeyError):
            pass

    pub = roslibpy.Topic(client, '/vlm/target', 'std_msgs/String')
    sub = roslibpy.Topic(client, '/vlm/target', 'std_msgs/String')
    pub.advertise()
    sub.subscribe(on_msg)
    time.sleep(1.0)

    print('publishing test messages on /vlm/target for %d s ...' % args.seconds)
    n = 0
    end = time.time() + args.seconds
    while time.time() < end and client.is_connected:
        n += 1
        msg = {'label': 'test', 'found': True, 'conf': 0.0, 'box': [0, 0, 0, 0], 'cx': 320, 'cy': 240,
               'img_w': 640, 'img_h': 480, 'frame_time': time.time(), 'sent_time': time.time(), 'seq': n}
        pub.publish(roslibpy.Message({'data': json.dumps(msg)}))
        time.sleep(0.5)
        if n % 10 == 0:
            recent = delays[-10:]
            if recent:
                print('  sent %d, received back %d, round trip median %.0f ms (max %.0f ms)'
                      % (n, len(delays), statistics.median(recent), max(recent)))
            else:
                print('  sent %d, nothing received back yet' % n)

    print('done: sent %d, received back %d' % (n, len(delays)))
    if delays:
        print('round trip: median %.0f ms, max %.0f ms' % (statistics.median(delays), max(delays)))
    pub.unadvertise()
    sub.unsubscribe()
    client.terminate()


if __name__ == '__main__':
    main()
