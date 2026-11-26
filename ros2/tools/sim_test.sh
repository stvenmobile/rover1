#!/bin/bash
# end-to-end check against fake_llc.py (run inside a Humble environment, from the workspace root)
source install/setup.bash
[ -f /deps/install/setup.bash ] && source /deps/install/setup.bash
odom_x() { ros2 topic echo --once /odom nav_msgs/msg/Odometry | sed -n "9,10p" | tr -s ' \n' ' '; echo; }
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
python3 tools/fake_llc.py /tmp/rover1_fake > /tmp/fake.log 2>&1 &
FAKE=$!
sleep 1
timeout 32 ros2 launch rover1_bringup rover1.launch.py device:=/tmp/rover1_fake > /tmp/launch.log 2>&1 &
sleep 8
echo "drive 0.2 m/s for 4 s";  timeout 4 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" >/dev/null; odom_x
echo "simulate ESP32 reboot";   kill -USR1 $FAKE; sleep 1; odom_x
echo "drive 0.2 m/s for 4 s";  timeout 4 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" >/dev/null; odom_x
echo "0.001 m/s (inside deadband) for 2 s"; timeout 2 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.001}}" >/dev/null; odom_x
sleep 10
grep -E "Rover1System|ERROR|WARN" /tmp/launch.log | grep -v -E "Deprecated|FIFO"
kill $FAKE
