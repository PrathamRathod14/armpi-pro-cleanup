#!/bin/bash
# Stop fetch_blue.py and the front camera, and make sure the wheels are stopped.
source ~/armpi_pro/devel/setup.bash
pkill -INT -f fetch_blue.py; pkill -INT -f front_cam.py; pkill -INT -f wander.py
sleep 2
rostopic pub -1 /chassis_control/set_velocity chassis_control/SetVelocity "{velocity: 0.0, direction: 90.0, angular: 0.0}" > /dev/null
echo "Stopped. Last lines of the log:"
tail -n 15 ~/fetch_log.txt 2>/dev/null
