# Plan: YOLO-World on the laptop, grabbing on the robot

Goal: type the name of an object ("blue cube", "black cap") on the laptop, and the robot
finds it, lines the gripper up over it with inverse kinematics, grabs it and places it on
its left. No colour tuning.

Status as of 14 November 2026: phases 1–3 done; phase 4 written and simulated, robot test pending.

## Architecture

```
 LAPTOP (Windows, i5-1235U, no NVIDIA GPU)            ROBOT (Raspberry Pi 4, ROS Melodic)
 ------------------------------------------           -----------------------------------
 vlm_bridge.py                                         web_video_server :8080
   1. GET /snapshot?topic=/usb_cam/image_raw  <------  (already running)
   2. YOLO-World: find "blue cube" in the picture
   3. publish box as JSON on /vlm/target      ------>  rosbridge websocket :9090
                                                         (already running)
                                                       fetch_blue.py vlm "blue cube"
                                                         - subscribes /vlm/target
                                                         - local tracker between updates
                                                         - IK line-up, wrist turn, grab,
                                                           place left, check, retry
```

Why split it this way:

- The Pi 4 is too slow for YOLO-World; the laptop CPU is about 10 times faster.
- The network adds delay (picture travels to the laptop, result travels back). So the laptop
  only says *where the object is*; the robot keeps its fast local loop (about 15 per second)
  that steers the arm, and corrects itself whenever a new laptop result arrives.
- Nothing new is installed on the robot: the camera server and rosbridge already run.

## Message on /vlm/target (std_msgs/String, JSON)

```json
{"label": "blue cube", "found": true, "conf": 0.62,
 "box": [x1, y1, x2, y2], "cx": 331, "cy": 402,
 "img_w": 640, "img_h": 480, "frame_time": 1794579000.12, "sent_time": 1794579000.41}
```

`found: false` is sent when nothing matches, so the robot knows the laptop is alive.
`sent_time - frame_time` is the processing delay, used to judge how stale a result is.

## Phases

### Phase 1 — Laptop setup and speed test (now)

1. Python environment in `vlm/.venv` with PyTorch (CPU), Ultralytics, OpenVINO, roslibpy, OpenCV, CLIP.
2. Download YOLO-World v2 small (`yolov8s-worldv2.pt`, about 25 MB).
3. Run it on real robot camera pictures with prompts "blue cube", "black cap", "pink highlighter".
4. Measure time per picture: plain PyTorch vs exported to OpenVINO; image size 640 vs 480 vs 320.
5. Check the robot's rosbridge on port 9090 answers (done: it accepts websocket connections).

Done when: the blue cube is found with confidence above 0.3 in robot pictures, and one setting
runs at 3 or more pictures per second.

**Results (13 November 2026), laptop CPU only (PyTorch; the OpenVINO export failed because of
Windows' path-length limit in this folder, and is not needed at these speeds):**

| Model | Prompt | Image size | Speed | Blue cube found in 3 robot pictures |
|---|---|---|---|---|
| YOLO-World small | "blue cube" | 640 | 7.9/s | No (nothing above 0.05) |
| YOLO-World small | "blue square" | 640 | ~7/s | Weakly (0.09) |
| **YOLO-World medium** | **"blue square"** | **480** | **4.5/s** | **Yes, 3 of 3, confidence 0.17–0.28, box exactly on the cube** |
| YOLO-World medium | "blue square" | 640 | 3.1/s | Yes, 3 of 3, confidence 0.25–0.32 (0.43 in one test) |
| YOLO-World medium | "blue square", "black cap" | 480 | 3.8/s | Yes, 3 of 3, 0.26–0.38 |

Findings:

- The prompt matters a lot: seen from above, the cube looks like a flat square, so "blue square"
  works and "blue cube" does not.
- The medium model is needed; the small one barely sees the cube.
- No false detections on the dark-blue jeans and jacket that fooled the colour filter.
- Chosen setting: medium model, image size 480, confidence threshold 0.15, about 4 pictures per second.
- Not yet tested: the black cap (it was not in the pictures).

### Phase 2 — Connection test

1. From the laptop, connect to `ws://10.238.54.20:9090` with roslibpy and list the robot's topics.
2. Publish a test message on `/vlm/target`; on the robot, `rostopic echo /vlm/target` shows it.

Done when: messages from the laptop appear on the robot within half a second.

**Results (13 November 2026): passed.** `vlm/phase2_test.py` connected to the robot's rosbridge in
0.15 s and saw all 67 robot topics (camera, wheels, arm servos). 24 of 24 test messages made the
round trip laptop -> robot -> laptop, median 37 ms, maximum 113 ms. On the robot,
`rostopic echo /vlm/target` showed the messages arriving. The network delay is small next to
YOLO-World's ~220 ms per picture.

### Phase 3 — Laptop bridge script (`vlm/vlm_bridge.py`)

1. Loop: fetch snapshot, run YOLO-World with the typed prompt, publish the result.
2. Command line: `python vlm_bridge.py --prompt "blue cube" --robot 10.238.54.20`.
3. Optional window on the laptop showing the picture with the detected box.
4. Prints pictures per second and delay.
5. Keeps the best box only (highest confidence above a threshold); reconnects if the robot drops.

Done when: it runs for 5 minutes without stopping, and the box follows the cube when moved by hand.

**Results so far (13 November 2026): stability passed, tracking not yet tested.**

- Downloading the next picture in the background while detecting doubled the speed (2 -> 4 per second).
- 5-minute run: 1000 pictures, no crash, every result sent to the robot, 2.6–4.7 pictures per second
  (average 289 ms per picture), result reaching the robot about 0.5–0.7 s after the picture was taken.
- The blue cube was not on the table during the run, so "box follows the cube" is still to be tested.
- No false detections in 1000 pictures, even with the laptop (with a small blue sticker), a person
  and dark-blue clothing in view.
- 90-second run with the cube on the table (prompt "blue square", also naming "hand", threshold 0.08):
  **found in 315 of 315 pictures (100%)**, confidence 0.23–0.45, position steady at (265, 388),
  3.5–4.1 pictures per second, result reaching the robot 0.5–0.6 s after the picture.
  The cube stayed in one place during the run, so following a moving cube is still untested;
  the robot's own tracker (phase 4) handles motion between laptop updates.
- While a hand held the cube, confidence dropped to 0.08–0.10: the grab should happen with
  hands away from the cube.

### Phase 4 — Robot `vlm` mode in `fetch_blue.py`

1. `python3 ~/fetch_blue.py vlm` (arm only, like `arm`) subscribes to `/vlm/target`.
2. Local tracking between laptop updates: when a laptop box arrives, cut a small template of the
   object from the current picture; follow it with OpenCV template matching each frame
   (works for any colour, unlike the HSV filter). Each new laptop box re-seeds the template.
3. Safety: if no laptop result for 3 seconds, the arm holds still; after 10 seconds, it gives up.
4. Everything after that is reused unchanged: IK line-up, wrist alignment, grab, place left,
   camera check, retry.
5. Wrist alignment needs the object's outline: use the HSV patch inside the laptop box when
   available, otherwise skip the wrist turn.

Done when: the blue cube is grabbed in arm-only mode with the laptop finding it.

**Status (14 November 2026): written, robot test pending.** Implemented as a step-by-step line-up
instead of a continuous template tracker: the robot waits for a laptop result whose picture was
taken after the arm stopped moving (`VLM_SETTLE`), moves toward the object, and reverses an axis if
the error grew. This avoids steering by half-second-old pictures. An offline simulation settled
over the cube in 5–7 steps in 9 of 9 cases (cube nearer, farther, and with the reach direction
reversed), final error 0.5–6 mm. Two robot test sessions were lost: the robot went offline mid-run,
and after the next boot its camera server and ROS bridge never started, most likely low battery.

### Phase 5 — Tests and comparison

| Test | Objects | Compare with |
|---|---|---|
| A | Blue cube alone | Colour version (`fetch_blue.py arm`) |
| B | Blue cube with red and wooden cubes nearby | Must never pick the wrong cube |
| C | Black cap | The black-colour attempt that chased the table edge |
| D | Person or dark clothing in view | Must not track them |
| E | Laptop disconnected mid-run | Arm must stop safely |

Record per test: grabbed (yes/no), wrong object (yes/no), time to grab, laptop pictures per second.

### Phase 6 — Optional, later

- Full mode with driving (`fetch_blue.py vlm` without arm-only) once phase 5 passes.
- A larger VLM (Florence-2, or a cloud model) for instructions like "pick up the thing next to the cup".

## Risks and how we handle them

| Risk | Effect | Handling |
|---|---|---|
| Laptop CPU too slow | Under 2 pictures per second | OpenVINO export, smaller image size (320/480), small model |
| Network delay or drop | Stale or missing boxes | Robot tracks locally; stops if the laptop goes quiet |
| `ubuntu.local` resolves to IPv6 | Laptop cannot reach the camera | Use the number address, `10.238.54.20` |
| rosbridge on the robot is old (Melodic) | roslibpy may not connect | Fallback: small HTTP endpoint on the robot instead of rosbridge |
| YOLO-World confuses objects | Wrong target | Confidence threshold, exact prompts, test B and D |
| Camera crash on the robot | No pictures for either side | Same as before: reboot; laptop script reports "no picture" |
| Gripper limits | Cap too big to grip | Prefer pinchable objects; grip the brim later |

## Who does what

| Step | Laptop (done from here) | You, at the robot |
|---|---|---|
| Phase 1 | Install, download, test, measure | Keep the robot on with the camera running |
| Phase 2 | Connect and publish | Run `rostopic echo /vlm/target` and paste the output |
| Phase 3 | Write and run the bridge | Move the cube around while it runs |
| Phase 4 | Write the robot code | Paste it onto the robot, run arm-only tests |
| Phase 5 | Collect results | Set up each test scene |

## Second camera: ACEBOTT QD003 AI vision module (found 14 November 2026)

- The module is a Kendryte K210 board running CanMV (MicroPython) with a GC2145 camera sensor
  and its own small screen. It is **not** a USB webcam: over USB it appears only as a serial port
  (CH340 / "QinHeng HL-340", ID 1a86:7523), never as `/dev/video*`.
- On the robot it is `/dev/ttyUSB0`. Confirmed by resetting it over that port and reading its
  startup messages (`[CANMV] ... find gc2145`).
- The robot's arm servos use `/dev/ttyS0` (the Pi's own serial pins), not USB, so the module
  does not conflict with them and can be plugged in at any time, in any USB-A port.
- The screen works on any USB power; the robot only sees the module when the cable and port carry data.
- Not used yet. Options for later: (A) a CanMV script on the module finds the object itself and
  sends its position over serial; (B) the module sends compressed pictures over serial as a second
  view for YOLO-World; (C) use a standard USB webcam instead.
