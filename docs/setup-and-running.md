# Setup and running

## 1. Connect to the robot

1. Turn on the phone hotspot (2.4 GHz, WPA2), then the robot, and wait for the beep.
2. Connect the laptop to the same hotspot.
3. Log in and load the ROS workspace:

```bash
ssh ubuntu@ROBOT_ADDRESS
source ~/armpi_pro/devel/setup.bash
```

`ROBOT_ADDRESS` is the robot's number address on the hotspot (run `hostname -I` on the robot to see it; it has been `10.179.98.20` and `10.238.54.20`). `ubuntu.local` may not work on Windows.

If the robot cannot join the hotspot, it starts its own network (`HW-xxxxxxxx`). Connect the laptop to that and use `ssh ubuntu@192.168.149.1`. The robot's Wi-Fi settings are in `/etc/Hiwonder/wifi.yaml` (template: `robot/wifi.yaml.example`).

## 2. Copy the scripts to the robot

From the laptop, in this repository's folder:

```bash
scp robot/*.py robot/*.sh ubuntu@ROBOT_ADDRESS:~/
```

On the robot the scripts live in the home folder (`~/fetch_blue.py` and so on).

## 3. Teach the arm poses (once)

```bash
python3 ~/cleanup.py teach
```

Move the arm with the keys and save:

| Key | Saves |
|---|---|
| `1` | **LOOK** pose: arm raised, camera seeing the floor ahead, gripper open |
| `2` | Ready pose: arm down, gripper open |
| `c` | Grab spot: where an object sits in the picture when the gripper can take it |
| `3` | Grip pose: grabbing height and how far the gripper closes |
| `4` | **DROP** pose: arm turned round over the basket, cube held just above the opening |
| `v` / `x` | Show what is saved / quit |

Movement keys: `w s a d q e` wheels, `j/l i/k u/o y/h n/m` arm joints, `r/f` gripper. `fetch_blue.py` needs at least the LOOK and DROP poses.

Then measure the grab point once (arm only, nothing drives):

```bash
python3 ~/fetch_blue.py calibgrab
```

## 4. Run it

Charge the battery first (at least 7.5 V to drive; check with `rostopic echo -n 5 /sensor/battery`, value in millivolts).

**Recommended order for testing:** `look` → `arm` → `frontarm` → `front`.

```bash
python3 ~/fetch_blue.py look       # detection test: nothing moves
python3 ~/fetch_blue.py arm        # arm only: grabs a cube placed 12-25 cm in front
python3 ~/front_cam.py             # front camera (keep it running in a second window)
python3 ~/fetch_blue.py frontarm   # front camera finds, arm grabs, no driving
python3 ~/fetch_blue.py front      # front camera finds, wheels drive, arm grabs (floor only)
python3 ~/fetch_blue.py            # arm camera finds, wheels drive, arm grabs (floor only)
```

Modes that drive must be used on the floor, never on a table.

### Run in the background (survives a Wi-Fi drop)

```bash
bash ~/run_robot.sh            # front mode
bash ~/run_robot.sh frontarm   # arm only
```

This starts the front camera and `fetch_blue.py` in the background and follows the log. Ctrl+C only stops following the log; the robot keeps going.

### Stop

```bash
bash ~/stop_robot.sh
```

Stops `fetch_blue.py`, `front_cam.py` and `wander.py`, then sends the wheels a zero speed. Logs: `~/fetch_log.txt`, `~/front_cam.log`.

### Watch the cameras

In the laptop browser (or open `docs/cameras.html`):

- Arm camera: `http://ROBOT_ADDRESS:8080/stream_viewer?topic=/usb_cam/image_raw`
- Arm camera with detection drawn on top (while `fetch_blue.py` runs): `...?topic=/cleanup/image`
- Front camera (while `front_cam.py` runs): `...?topic=/front_cam/image`

## 5. Laptop vision (`vlm` mode)

Once, on the Windows laptop: double-click `install_deps.bat`, or in the `vlm` folder:

```bash
python -m venv .venv
.venv\Scripts\pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
.venv\Scripts\pip install ultralytics roslibpy opencv-python requests numpy git+https://github.com/ultralytics/CLIP.git
```

Each time:

```bash
# laptop, in the vlm folder
.venv\Scripts\python vlm_bridge.py --robot ROBOT_ADDRESS --prompt "blue square" --also "hand" --conf 0.08

# robot
python3 ~/fetch_blue.py vlm
```

In `vlm` mode the robot ignores Ctrl+C while waiting for the laptop. Stop it with **Ctrl+\\** or `bash ~/stop_robot.sh`. Stop the laptop bridge by pressing `q` in its picture window.

## 6. Other tools

```bash
python3 ~/teleop_keys.py        # drive and move the arm by keyboard
python3 ~/detect_objects.py     # colour detection only, for tuning colour ranges
python3 ~/wander.py 60          # drive around for 60 s (open floor only, no obstacle sensor)
```

## 7. Shut down

Always run `sudo halt` and wait a few seconds before switching the power off.

## 8. Save changes to GitHub

From PowerShell on the laptop:

```powershell
cd Desktop\armpi-pro-cleanup
git add -A
git commit -m "what you changed"
git push
```

If `git push` asks you to sign in, a GitHub login opens in the browser. In PowerShell use `$HOME`, not `%USERPROFILE%`.
