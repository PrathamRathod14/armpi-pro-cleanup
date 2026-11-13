"""Phase 3: the laptop's "eyes" for the robot.

Fetches pictures from the robot's camera, finds the named object with YOLO-World,
and publishes where it is on /vlm/target (std_msgs/String, JSON) through the robot's
rosbridge. The robot's fetch_blue.py (vlm mode) uses that to steer the arm.

    python vlm_bridge.py --prompt "blue square"
    python vlm_bridge.py --prompt "black cap" --conf 0.2
    python vlm_bridge.py --prompt "blue square" --also "red square" "wooden block"   # helps tell them apart

A window shows the camera picture with the detection; press q in it (or Ctrl+C here) to stop.

Message on /vlm/target:
  {"label", "found", "conf", "box": [x1, y1, x2, y2], "cx", "cy", "img_w", "img_h",
   "frame_time", "sent_time", "seq", "fps"}
found is false when nothing matches (so the robot knows the laptop is still alive).
"""
import argparse
import json
import os
import statistics
import threading
import time

import cv2
import numpy as np
import requests
import roslibpy
from ultralytics import YOLOWorld

HERE = os.path.dirname(os.path.abspath(__file__))


class Grabber(threading.Thread):
    """Downloads robot camera pictures continuously in the background, keeping only the newest,
    so detection never waits for a download."""

    def __init__(self, url):
        threading.Thread.__init__(self, daemon=True)
        self.url = url
        self.session = requests.Session()
        self.lock = threading.Lock()
        self.frame, self.frame_time, self.count, self.running = None, 0.0, 0, True

    def run(self):
        while self.running:
            t0 = time.time()
            try:
                r = self.session.get(self.url, timeout=5)
                img = cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_COLOR)
            except requests.RequestException:
                img = None
            if img is None:
                time.sleep(0.5)
                continue
            with self.lock:
                self.frame, self.frame_time = img, t0
                self.count += 1

    def latest(self, after_count, timeout=5.0):
        """Wait for a picture newer than `after_count`; returns (picture, time taken, count)."""
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if self.count > after_count and self.frame is not None:
                    return self.frame, self.frame_time, self.count
            time.sleep(0.005)
        return None, 0.0, after_count


class RobotLink(object):
    """Connection to the robot's rosbridge, reconnecting if it drops."""

    def __init__(self, host):
        self.host = host
        self.client = None
        self.pub = None
        self.last_try = 0.0

    def ok(self):
        if self.client is not None and self.client.is_connected:
            return True
        if time.time() - self.last_try < 3.0:
            return False
        self.last_try = time.time()
        try:
            if self.client is not None:
                try:
                    self.client.terminate()
                except Exception:
                    pass
            self.client = roslibpy.Ros(host=self.host, port=9090)
            self.client.run(timeout=5)
            self.pub = roslibpy.Topic(self.client, '/vlm/target', 'std_msgs/String')
            self.pub.advertise()
            print('connected to robot rosbridge ws://%s:9090' % self.host)
            return True
        except Exception as e:
            print('robot rosbridge not reachable (%s) - retrying' % e.__class__.__name__)
            return False

    def send(self, msg):
        if self.ok():
            self.pub.publish(roslibpy.Message({'data': json.dumps(msg)}))
            return True
        return False

    def close(self):
        try:
            if self.pub is not None:
                self.pub.unadvertise()
            if self.client is not None:
                self.client.terminate()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', default='10.238.54.20', help='robot IP address')
    ap.add_argument('--prompt', default='blue square', help='what to look for (the target)')
    ap.add_argument('--also', nargs='*', default=[], help='other objects to name, so they are not confused with the target')
    ap.add_argument('--model', default='yolov8m-worldv2.pt')
    ap.add_argument('--imgsz', type=int, default=480)
    ap.add_argument('--conf', type=float, default=0.15, help='minimum confidence for the target')
    ap.add_argument('--no-show', action='store_true', help='do not open the picture window')
    ap.add_argument('--seconds', type=float, default=0, help='stop after this long (0 = run until stopped)')
    args = ap.parse_args()

    classes = [args.prompt] + [a for a in args.also if a != args.prompt]
    os.makedirs(os.path.join(HERE, 'models'), exist_ok=True)
    cwd = os.getcwd()
    os.chdir(os.path.join(HERE, 'models'))
    model = YOLOWorld(args.model)
    model.set_classes(classes)
    os.chdir(cwd)
    print('looking for "%s"%s with %s at size %d, confidence >= %.2f'
          % (args.prompt, (' (also naming: %s)' % ', '.join(classes[1:])) if classes[1:] else '',
             args.model, args.imgsz, args.conf))

    grabber = Grabber('http://%s:8080/snapshot?topic=/usb_cam/image_raw' % args.robot)
    grabber.start()
    link = RobotLink(args.robot)
    link.ok()

    seq, found_n, times, ages, t_report = 0, 0, [], [], time.time()
    start, last_count = time.time(), 0
    try:
        while not args.seconds or time.time() - start < args.seconds:
            t0 = time.time()
            img, frame_time, last_count = grabber.latest(last_count)
            if img is None:
                print('no picture from the robot camera - is it running?')
                continue
            res = model.predict(img, imgsz=args.imgsz, conf=0.05, verbose=False)[0]
            best = None
            for box, cls, conf in zip(res.boxes.xyxy.tolist(), res.boxes.cls.tolist(), res.boxes.conf.tolist()):
                if int(cls) == 0 and conf >= args.conf and (best is None or conf > best[1]):
                    best = (box, conf)
            seq += 1
            h, w = img.shape[:2]
            elapsed = time.time() - t0
            times.append(elapsed)
            fps = 1.0 / statistics.mean(times[-10:])
            msg = {'label': args.prompt, 'found': best is not None, 'conf': 0.0, 'box': [0, 0, 0, 0],
                   'cx': 0, 'cy': 0, 'img_w': w, 'img_h': h, 'frame_time': frame_time,
                   'sent_time': time.time(), 'seq': seq, 'fps': round(fps, 2)}
            if best:
                (x1, y1, x2, y2), conf = best
                found_n += 1
                msg.update(conf=round(conf, 3), box=[int(x1), int(y1), int(x2), int(y2)],
                           cx=int((x1 + x2) / 2), cy=int((y1 + y2) / 2))
            sent = link.send(msg)
            ages.append(msg['sent_time'] - frame_time)

            if not args.no_show:
                view = img.copy()
                for box, cls, conf in zip(res.boxes.xyxy.tolist(), res.boxes.cls.tolist(), res.boxes.conf.tolist()):
                    x1, y1, x2, y2 = [int(v) for v in box]
                    target = int(cls) == 0 and conf >= args.conf
                    col = (0, 200, 0) if target else (150, 150, 150)
                    cv2.rectangle(view, (x1, y1), (x2, y2), col, 3 if target else 1)
                    cv2.putText(view, '%s %.2f' % (classes[int(cls)], conf), (x1, max(15, y1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
                cv2.putText(view, '%.1f pictures/s  %s' % (fps, 'sent to robot' if sent else 'ROBOT NOT CONNECTED'),
                            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255) if not sent else (0, 160, 0), 2)
                cv2.imshow('Robot camera - YOLO-World (press q to stop)', view)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if time.time() - t_report > 2.0:
                t_report = time.time()
                where = 'at (%d, %d) conf %.2f' % (msg['cx'], msg['cy'], msg['conf']) if best else 'not found'
                print('  %5.1f pictures/s | result %.0f ms after the picture | %s %s | found in %d of %d | %s'
                      % (fps, 1000 * statistics.mean(ages[-10:]), args.prompt, where, found_n, seq,
                         'sent' if sent else 'NOT SENT'))
    except KeyboardInterrupt:
        pass
    finally:
        grabber.running = False
        link.close()
        cv2.destroyAllWindows()
        if times:
            print('stopped: %d pictures, target found in %d (%.0f%%), average %.0f ms per picture'
                  % (seq, found_n, 100.0 * found_n / max(1, seq), 1000 * statistics.mean(times)))


if __name__ == '__main__':
    main()
