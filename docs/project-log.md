# Project log

What was built, day by day. The full detail is in the git history (`git log`).

## Goal

A Hiwonder ArmPi Pro that cleans up on its own: it finds blue and red cubes, drives to them, picks them up and drops them in the basket on its back. Later goal: say the name of any object and have the robot fetch it.

## 13 November 2026: first working grab

- First clean-up scripts: colour detection, teach mode for arm poses, arm-only pick test, calibration, and a simple fetch mode (`cleanup.py`).
- First grab with inverse kinematics (`grasp_ik.py`), then fixes so it follows the same object and only grabs when on target.
- New main program `fetch_blue.py`: find the blue cube anywhere, drive to it, line up with inverse kinematics, grab, place it.
- Stable detection (strict and loose colour tests, shape checks, tracker), wrist turned to match the cube's sides, firmer grip.
- **Status:** the arm-only grab works and detection is stable.
- Laptop vision started: YOLO-World phase 1 benchmark. Result: medium model, prompt `"blue square"`, 4.5 pictures per second.
- Phase 2 passed: the laptop publishes to the robot through rosbridge, 37 ms round trip.
- `vlm_bridge.py` written; a 5-minute run was stable with no false detections.

## 14 November 2026: laptop vision on the robot side

- Bridge found the cube in 315 of 315 pictures.
- `vlm` mode added to `fetch_blue.py`: step-by-step line-up using the laptop's results. A simulation settled over the cube in 5–7 steps.
- The ACEBOTT K210 vision module identified as a possible second camera (it is a serial device, not a webcam).

## 16 November 2026: front camera, basket, safety

- Grab-point calibration mode (`calibgrab`).
- Cube now dropped in the basket on the robot's back instead of on its left.
- Front camera: `front_cam.py` runs a detection program on the K210 module and publishes it as a ROS camera; `front` and `frontarm` modes; two-camera browser page.
- Gentle wheel starts and a battery check before driving (sudden starts restarted the Pi).
- Check that the cube is in the gripper, and grab again on the spot if it missed.
- "Arrange" step: exact centring before grabbing, and a settle squeeze.
- Red cubes picked as well as blue, one after another until none is left.
- Robot voice (espeak, or beeps without a speaker).
- `wander.py`: drives around on its own.
- Smooth driving: no instant wheel reversals, lock onto one cube, slower proportional turning.
- Safe driving: arm camera paused while driving, battery dip watch, `run_robot.sh` and `stop_robot.sh` with a log that survives a crash.
- Battery reading uses the middle of several readings; no driving below 7.5 V.
- Battery measured at 7.32 V, so driving was refused.

## 17 November 2026: low battery, grab tuning

- The arm behaved badly with unchanged code; the battery was at about 6.2 V with dips to 5.2 V (critically low).
- `fetch_blue.py` changes (committed 19 November): `REACH_TRIM` and `BASE_TRIM` to correct where the gripper lands, one slow close (`GRIP_SPEED`), voice removed, and the "is it held" check made optional (`HOLD_CHECK = False`) because it was unreliable.
- Written in a separate session and kept outside this repository: `patch_autograb.py` (learns the grab point by itself on the first grab) and `cube_fetch.py` (a standalone version that drops cubes behind the robot). Neither has been run on the robot yet.

## 18–19 November 2026: documentation and clean-up

- Plan page updated with the front-mode sequence for blue and red cubes.
- `install_deps.bat` (installs the laptop vision software) and `open_docs.bat` (opens the docs pages) added.
- All changes pushed to GitHub.
- This documentation folder written, with architecture and workflow diagrams.
- Removed files the current code doesn't use: the 26 one-off `patch_*.py` scripts (their changes are already in `fetch_blue.py`) and the early experiments `grasp_ik.py` and `grasp_black.py`. They are still in the git history.

## Current status

| Part | Status |
|---|---|
| Colour detection (arm camera) | Works, stable |
| Arm-only grab (`arm`) | Works, but not every time |
| Front camera finding (`frontarm`) | Works |
| Driving to the cube (`front`, `full`) | Written; limited by battery so far |
| Drop in the basket | Works |
| Laptop vision (`vlm`) | Laptop side works; robot grab written and simulated, not yet tested on the robot |
| Self-learning grab point, `cube_fetch.py` | Written, not run, not in this repository |

## Next steps

1. Test with a fully charged battery: `arm`, then `frontarm`, then `front`.
2. Tune `REACH_TRIM` and `BASE_TRIM` until the grab lands in the middle every time.
3. Test `vlm` mode on the robot (tests A–E in [vlm-plan.md](vlm-plan.md)).
4. Add `cube_fetch.py` and `patch_autograb.py` to the repository once they have been tested.
