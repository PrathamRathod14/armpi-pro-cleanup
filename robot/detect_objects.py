#!/usr/bin/env python3
# Clean-up robot: find coloured objects in the camera image (no movement).
import time
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Image

# HSV colour ranges (hue 0-179). Tune these for your lighting.
COLOURS = {
    'red':    [((0, 120, 70), (8, 255, 255)), ((174, 120, 70), (179, 255, 255))],
    'pink':   [((150, 70, 100), (173, 255, 255))],
    'yellow': [((20, 70, 90), (36, 255, 255))],
    'green':  [((40, 80, 60), (85, 255, 255))],
    'blue':   [((95, 110, 60), (130, 255, 255))],
}
DRAW = {'red': (0, 0, 255), 'pink': (180, 105, 255), 'yellow': (0, 255, 255),
        'green': (0, 255, 0), 'blue': (255, 0, 0)}
MIN_AREA = 600   # ignore blobs smaller than this many pixels

last_print = 0.0

def find_objects(img):
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    found = []
    for name, ranges in COLOURS.items():
        mask = None
        for lo, hi in ranges:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        for c in contours:
            area = cv2.contourArea(c)
            if area >= MIN_AREA:
                x, y, w, h = cv2.boundingRect(c)
                found.append((area, name, x + w // 2, y + h // 2, (x, y, w, h)))
    return sorted(found, reverse=True)

def on_image(msg):
    global last_print
    img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return
    objects = find_objects(img)
    for area, name, cx, cy, (x, y, w, h) in objects:
        cv2.rectangle(img, (x, y), (x + w, y + h), DRAW[name], 2)
        cv2.putText(img, '%s (%d,%d)' % (name, cx, cy), (x, max(15, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, DRAW[name], 2)
    out = Image()
    out.header = msg.header
    out.height, out.width = img.shape[:2]
    out.encoding = 'bgr8'
    out.step = out.width * 3
    out.data = img.tobytes()
    pub.publish(out)
    if time.time() - last_print > 0.5:
        last_print = time.time()
        if objects:
            summary = ', '.join('%s at (%d,%d) area %d' % (n, cx, cy, a) for a, n, cx, cy, _ in objects[:4])
            print('%d seen: %s' % (len(objects), summary))
        else:
            print('No objects seen')

rospy.init_node('detect_objects')
pub = rospy.Publisher('/cleanup/image', Image, queue_size=1)
rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, on_image,
                 queue_size=1, buff_size=2 ** 24)
print('Detecting... view at http://ubuntu.local:8080/stream_viewer?topic=/cleanup/image   (Ctrl+C to stop)')
rospy.spin()
