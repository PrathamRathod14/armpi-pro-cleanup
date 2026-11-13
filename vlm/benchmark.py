"""Phase 1: run YOLO-World on robot camera pictures and measure speed.

    python benchmark.py --robot 10.238.54.20 --prompts "blue cube" "black cap" "pink highlighter"

Saves annotated pictures to vlm/snapshots/ and prints detections and timings
for PyTorch and OpenVINO at several image sizes.
"""
import argparse
import os
import time

import cv2
import numpy as np
import requests
from ultralytics import YOLO, YOLOWorld

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, 'models')
SNAPS = os.path.join(HERE, 'snapshots')


def fetch(robot):
    url = 'http://%s:8080/snapshot?topic=/usb_cam/image_raw' % robot
    r = requests.get(url, timeout=8)
    r.raise_for_status()
    img = cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError('robot sent something that is not a picture')
    return img


def time_model(model, img, imgsz, runs=8):
    model.predict(img, imgsz=imgsz, conf=0.05, verbose=False)          # warm-up
    t0 = time.perf_counter()
    for _ in range(runs):
        res = model.predict(img, imgsz=imgsz, conf=0.05, verbose=False)[0]
    return (time.perf_counter() - t0) / runs, res


def describe(res, prompts):
    out = []
    for box, cls, conf in zip(res.boxes.xyxy.tolist(), res.boxes.cls.tolist(), res.boxes.conf.tolist()):
        x1, y1, x2, y2 = [int(v) for v in box]
        out.append('%s %.2f at (%d,%d)-(%d,%d)' % (prompts[int(cls)], conf, x1, y1, x2, y2))
    return out or ['nothing']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', default='10.238.54.20')
    ap.add_argument('--prompts', nargs='+', default=['blue cube', 'black cap', 'pink highlighter'])
    ap.add_argument('--image', help='use a saved picture instead of the robot camera')
    args = ap.parse_args()
    os.makedirs(MODELS, exist_ok=True)
    os.makedirs(SNAPS, exist_ok=True)

    img = cv2.imread(args.image) if args.image else fetch(args.robot)
    stamp = time.strftime('%H%M%S')
    cv2.imwrite(os.path.join(SNAPS, 'input_%s.jpg' % stamp), img)
    print('picture: %dx%d' % (img.shape[1], img.shape[0]))

    os.chdir(MODELS)                                   # weights download here
    model = YOLOWorld('yolov8s-worldv2.pt')
    model.set_classes(args.prompts)

    print('\n--- PyTorch (CPU) ---')
    for imgsz in (640, 480, 320):
        sec, res = time_model(model, img, imgsz)
        print('imgsz %d: %.0f ms  (%.1f per second)  -> %s' % (imgsz, sec * 1000, 1 / sec, '; '.join(describe(res, args.prompts))))
        if imgsz == 640:
            cv2.imwrite(os.path.join(SNAPS, 'pytorch_%s.jpg' % stamp), res.plot())

    print('\n--- OpenVINO (Intel CPU) ---')
    for imgsz in (640, 480, 320):
        path = model.export(format='openvino', imgsz=imgsz, half=False, verbose=False)
        ov = YOLO(path, task='detect')
        sec, res = time_model(ov, img, imgsz)
        print('imgsz %d: %.0f ms  (%.1f per second)  -> %s' % (imgsz, sec * 1000, 1 / sec, '; '.join(describe(res, args.prompts))))
        if imgsz == 640:
            cv2.imwrite(os.path.join(SNAPS, 'openvino_%s.jpg' % stamp), res.plot())


if __name__ == '__main__':
    main()
