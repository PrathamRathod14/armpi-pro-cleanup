# ArmPi Pro clean-up robot

A Hiwonder ArmPi Pro finds a blue cube, drives to it, picks it up and puts it in the basket
on its back. A front camera (ACEBOTT K210 AI vision module) finds the cube; the arm camera
lines up the grab.

**Status (13 November 2026):** the main program is `robot/fetch_blue.py`. It finds the blue
cube with colour and shape checks, lines the gripper up over it with Hiwonder's inverse
kinematics, turns the wrist to match the cube's sides, grabs it and places it on the left.
Arm-only mode (`fetch_blue.py arm`) runs on the robot: detection is stable and the grab works,
but not every time yet. The driving mode has not been tested with the IK grab.

Laptop vision: a YOLO-World model on the laptop (`vlm/vlm_bridge.py`) finds the object by name
and sends its position to the robot, where `fetch_blue.py vlm` grabs it. Phases 1–3 (model,
connection, bridge) work; the robot grab with it is written but not yet tested on the robot.
See `docs/vlm-plan.md`.

## Robot

| Part | Details |
|---|---|
| Computer | Raspberry Pi 4, Ubuntu 18.04, ROS Melodic |
| Camera | One USB colour camera on the arm |
| Arm | Six bus servos: 1 gripper, 2 wrist rotate, 3 wrist, 4 elbow, 5 shoulder, 6 base |
| Base | Mecanum wheels (forward, sideways, turn on the spot) |
| Other | Buzzer, RGB LEDs, battery voltage. No lidar, depth or distance sensor |

## Files

| Path | Purpose |
|---|---|
| `robot/fetch_blue.py` | **Main program.** Stable blue detection, IK line-up, wrist alignment, grab, check the cube is held (grab again if not), place in the basket, retry. Modes: `look` (detection test), `arm` (wheels never move), no argument (search and drive) |
| `vlm/vlm_bridge.py` | Laptop: fetches robot camera pictures, finds the named object with YOLO-World, sends its position to the robot on `/vlm/target` |
| `vlm/benchmark.py`, `vlm/phase2_test.py` | Laptop: model speed test, and robot connection test |
| `docs/vlm-plan.md` | Laptop vision plan with the results of each phase |
| `robot/front_cam.py` | Runs a detection program on the ACEBOTT K210 module and publishes `/front_cam/image` and `/front_cam/target` |
| `docs/cameras.html` | Shows the arm camera and the front camera side by side in a browser |
| `robot/grasp_ik.py` | Earlier IK grab (arm only), kept for reference |
| `robot/grasp_black.py` | Same IK grab looking for a dark object (the black cap test) |
| `robot/cleanup.py` | The clean-up program: colour detection, teach mode, arm-only pick test, calibration, and the autonomous fetch |
| `robot/patch_holdcheck.py` | Updates `~/fetch_blue.py` on the robot: after grabbing, the arm camera checks the cube is in the gripper and grabs again on the spot if it missed; only a held cube goes to the basket |
| `robot/run_robot.sh`, `robot/stop_robot.sh` | Start the front camera and `fetch_blue.py` (default `front`) so they keep running if Wi-Fi drops, with a log in `~/fetch_log.txt`; stop everything and the wheels |
| `robot/wander.py` | Drives around on its own for a set time with gentle starts (no obstacle sensor) |
| `robot/patch_voice.py` | Updates `~/fetch_blue.py`: the robot says what it is doing ("Got it! I picked up the cube.") with espeak and a speaker, or beeps with its buzzer if there is no speaker (`VOICE = 'auto' / 'beep' / 'off'`) |
| `robot/patch_red.py` | Updates `~/fetch_blue.py` and `~/front_cam.py`: red cubes are picked as well as blue ones, one after another until none is left (`PICK_RED = False` for blue only) |
| `robot/patch_arrange.py` | Updates `~/fetch_blue.py` (after `patch_holdcheck.py`): centres exactly over the cube before grabbing, and closes, loosens and closes again so the cube sits in the middle of the fingers |
| `robot/patch_fetch.py` | Adds the `calib` and `fetch` modes to an older `cleanup.py` already on the robot |
| `robot/teleop_keys.py` | Keyboard control of wheels, arm and gripper, with a saved home pose |
| `robot/detect_objects.py` | Colour detection only, no movement; useful for tuning colour ranges |
| `robot/wifi.yaml.example` | Template for the robot's Wi-Fi settings (`/etc/Hiwonder/wifi.yaml`) |
| `docs/cleanup-robot-plan.html` | Development plan and explainer with an animated simulation |
| `docs/cleanup-robot-simulation.html` | Step-by-step animation of the clean-up logic |

On the robot the scripts live in the home folder (`~/cleanup.py` and so on). Taught
positions are saved on the robot in `~/cleanup_poses.json` and `~/arm_home.json`; those
are not in this repository.

## Connecting

1. Turn on the phone hotspot (2.4 GHz, WPA2), then the robot, and wait for the beep.
2. Connect the laptop to the same hotspot.
3. Log in and load the ROS workspace:

```bash
ssh ubuntu@ubuntu.local
source ~/armpi_pro/devel/setup.bash
```

If the robot cannot join the hotspot, it starts its own (`HW-xxxxxxxx`); connect to that
and use `ssh ubuntu@192.168.149.1`. Wi-Fi settings are in `/etc/Hiwonder/wifi.yaml`
(see `robot/wifi.yaml.example`); `~/hiwonder-toolbox/wifi_conf.py` is not used by this
robot image.

## Copying the scripts to the robot

From the laptop, in this repository's folder:

```bash
scp robot/cleanup.py robot/teleop_keys.py robot/detect_objects.py ubuntu@ubuntu.local:~/
```

## Using it

```bash
python3 ~/fetch_blue.py look    # detection test: nothing moves
python3 ~/fetch_blue.py arm     # grab the blue cube with the arm only (safe on a table)
python3 ~/fetch_blue.py         # search and drive to the cube, then grab (floor only)
python3 ~/fetch_blue.py vlm     # grab using the laptop's YOLO-World detections (arm only)
python3 ~/front_cam.py          # front camera (keep running in a second window)
python3 ~/fetch_blue.py frontarm   # front camera finds the cube, arm grabs it into the basket (no driving)
python3 ~/fetch_blue.py front      # front camera finds the cube, wheels drive to it, arm grabs (floor only)
python3 ~/fetch_blue.py calibgrab  # measure the grab point once
```

Laptop side for `vlm` mode (Windows, in the `vlm` folder):

```bash
python -m venv .venv
.venv\Scripts\pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
.venv\Scripts\pip install ultralytics roslibpy opencv-python requests git+https://github.com/ultralytics/CLIP.git
.venv\Scripts\python vlm_bridge.py --prompt "blue square" --also "hand" --conf 0.08
```

Older modes in `cleanup.py`:

```bash
python3 ~/teleop_keys.py        # drive by keyboard
python3 ~/cleanup.py teach      # teach the arm positions
python3 ~/cleanup.py pick       # arm only: grab the cube under the gripper, place it left
python3 ~/cleanup.py calib      # once: record where the cube appears when it is in grabbing position
python3 ~/cleanup.py fetch      # find the blue cube, drive to it, pick it up, place it left
python3 ~/cleanup.py run        # older multi-object loop (Ctrl+C stops any mode)
```

`fetch` and `run` drive the robot: use them on the floor, never on a table.
On this robot a low gripper value means open (about 200) and a high value means closed.

Camera views in the laptop browser. If `ubuntu.local` does not load, use the robot's
number address instead (run `hostname -I` on the robot), for example `http://10.238.54.20:8080/...`:

- Camera: `http://ubuntu.local:8080/stream_viewer?topic=/usb_cam/image_raw`
- Clean-up view (while `cleanup.py` runs): `http://ubuntu.local:8080/stream_viewer?topic=/cleanup/image`

### Teach mode keys

| Key | Saves |
|---|---|
| `1` | Look pose: arm raised, camera seeing the floor ahead, gripper open |
| `2` | Ready pose: arm down, gripper open, fingertips at the bottom of the picture |
| `c` | Grab spot: where an object sits in the picture when the gripper can take it |
| `3` | Grip pose: grabbing height and how far the gripper closes |
| `4` | Drop pose: arm turned round over the basket on the robot's back, cube held just above the opening |
| `v` / `x` | Show what is saved / quit |

Movement keys are the same as in `teleop_keys.py`: `w s a d q e` for the wheels, `j/l i/k
u/o y/h n/m` for the arm joints, `r/f` for the gripper.

## How detection works

Colour thresholding with OpenCV, not a neural network. Each camera picture is blurred,
converted to HSV, and the pixels inside each colour's range are kept. Small specks are
removed, and each remaining patch gives a position and size. Distance comes from where the
patch sits in the picture: lower and bigger means closer.

## Shutting down

Always run `sudo halt` and wait a few seconds before switching the power off.
