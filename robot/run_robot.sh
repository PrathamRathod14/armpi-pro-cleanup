#!/bin/bash
# Start the front camera and fetch_blue.py so they keep running even if SSH / Wi-Fi drops.
#   bash ~/run_robot.sh            front mode (drives)
#   bash ~/run_robot.sh frontarm   arm only
# Logs (kept after a crash): ~/front_cam.log and ~/fetch_log.txt
# Ctrl+C only stops watching the log. To stop the robot:  bash ~/stop_robot.sh
source ~/armpi_pro/devel/setup.bash
MODE=${1:-front}
pkill -INT -f fetch_blue.py; pkill -INT -f front_cam.py; sleep 2
echo "=== $(date) start $MODE, Pi on for $(awk '{printf "%.0f", $1/60}' /proc/uptime) min ===" > ~/fetch_log.txt
setsid nohup python3 -u ~/front_cam.py > ~/front_cam.log 2>&1 < /dev/null &
sleep 3
setsid nohup python3 -u ~/fetch_blue.py "$MODE" >> ~/fetch_log.txt 2>&1 < /dev/null &
echo "Running $MODE. Ctrl+C stops watching (the robot keeps going). Stop the robot: bash ~/stop_robot.sh"
tail -f ~/fetch_log.txt
