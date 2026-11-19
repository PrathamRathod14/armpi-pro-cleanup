# Troubleshooting

## The arm shakes before it locks onto the cube

**Why:** the camera is on the arm. When the arm moves, the picture moves too, so the next picture shows the cube somewhere else and the robot corrects again, sometimes too far. The cube's detected centre also jitters a few pixels from picture to picture (light, shadows, shiny edges). A low battery makes the servos weak and wobbly, which adds to it.

**What the code already does:** ignores errors under 10 pixels, smooths the cube's position, moves in small steps, and only grabs after the cube stays centred for 10 pictures in a row.

**Fixes, easiest first:**

1. Charge the battery (8 V or more).
2. Make each step smaller: lower `X_GAIN` (now 0.03) and `Y_GAIN` (now 0.000015) in `fetch_blue.py`.
3. Ignore bigger jitter: raise `DEADBAND` (now 10 pixels).
4. Wait longer after each move: raise `TRACK_MS` (now 120 ms).

## The gripper lands next to the cube

- Lands short of the cube or past it: change `REACH_TRIM` (metres, + if short).
- Lands to one side: change `BASE_TRIM` (servo units, about 4 per degree).
- Run `python3 ~/fetch_blue.py calibgrab` again with a charged battery.

## The robot refuses to drive

The battery is below 7.5 V. Check with `rostopic echo -n 5 /sensor/battery` (millivolts) and charge it. Below about 6 V the pack can be damaged, so do not keep running.

## Bad arm movement that wasn't there yesterday

Almost always the battery. On 17 November the arm behaved badly with the code unchanged; the battery was at about 6.2 V with dips to 5.2 V.

## "No camera pictures - is the camera running?"

The arm camera node is not publishing. Check `rostopic list | grep usb_cam`. A reboot (`sudo reboot`) usually fixes it. After a low-battery shutdown the camera server and rosbridge sometimes do not start; charge, then reboot.

## "No results from the front camera"

Start `python3 ~/front_cam.py` first, in another window, or use `bash ~/run_robot.sh`, which starts both. Check the module is on `/dev/ttyUSB0` (`ls /dev/ttyUSB*`). Switching the module off and on brings back ACEBOTT's own program, so `front_cam.py` must be started again after that.

## "Teach the LOOK and DROP poses first"

Run `python3 ~/cleanup.py teach` and save poses `1` and `4`.

## The laptop can't reach the robot

- Both must be on the same hotspot.
- Use the number address, not `ubuntu.local` (Windows may resolve it to IPv6 or not at all).
- Check the ports: `http://ROBOT_ADDRESS:8080` must load in a browser (camera server), and port 9090 is rosbridge.

## `vlm` mode won't stop with Ctrl+C

That is on purpose while it waits for the laptop. Use **Ctrl+\\** or `bash ~/stop_robot.sh`.

## YOLO-World doesn't find the cube

- Use the prompt `"blue square"`, not `"blue cube"`.
- Lower `--conf` to 0.08.
- Keep hands away from the cube; a hand holding it dropped the confidence to about 0.1.

## The robot doesn't join the hotspot

It starts its own network `HW-xxxxxxxx`. Connect to that and log in at `192.168.149.1`, then fix `/etc/Hiwonder/wifi.yaml`.

## Git errors on Windows

- `Cannot find path '...%USERPROFILE%...'`: that is PowerShell. Use `cd Desktop\armpi-pro-cleanup` from your home folder, or `cd $HOME\Desktop\armpi-pro-cleanup`.
- `not a git repository`: you are not in the project folder; `cd` into it first.
- `CRLF will be replaced by LF`: harmless; Git normalises Windows line endings.
