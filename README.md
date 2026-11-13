# ArmPi Pro clean-up robot

A Hiwonder ArmPi Pro finds a blue cube, drives to it, picks it up and places it on the
floor to its own left. It uses only the camera on its arm.

**Status:** keyboard driving, arm control, camera view and teach mode work on the robot.
Colour detection works for red and blue cubes. The arm-only pick test runs. The
autonomous `fetch` mode has not been run on the robot yet.

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
| `robot/cleanup.py` | The clean-up program: colour detection, teach mode, arm-only pick test, calibration, and the autonomous fetch |
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
python3 ~/teleop_keys.py        # drive by keyboard
python3 ~/cleanup.py teach      # teach the arm positions
python3 ~/cleanup.py pick       # arm only: grab the cube under the gripper, place it left
python3 ~/cleanup.py calib      # once: record where the cube appears when it is in grabbing position
python3 ~/cleanup.py fetch      # find the blue cube, drive to it, pick it up, place it left
python3 ~/cleanup.py run        # older multi-object loop (Ctrl+C stops any mode)
```

`fetch` and `run` drive the robot: use them on the floor, never on a table.
On this robot a low gripper value means open (about 200) and a high value means closed.

Camera views in the laptop browser:

- Camera: `http://ubuntu.local:8080/stream_viewer?topic=/usb_cam/image_raw`
- Clean-up view (while `cleanup.py` runs): `http://ubuntu.local:8080/stream_viewer?topic=/cleanup/image`

### Teach mode keys

| Key | Saves |
|---|---|
| `1` | Look pose: arm raised, camera seeing the floor ahead, gripper open |
| `2` | Ready pose: arm down, gripper open, fingertips at the bottom of the picture |
| `c` | Grab spot: where an object sits in the picture when the gripper can take it |
| `3` | Grip pose: grabbing height and how far the gripper closes |
| `4` | Drop pose: arm turned left and lowered, object resting on the floor |
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
