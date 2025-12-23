#!/bin/bash
source /opt/ros/noetic/setup.bash; source /home/hightorque/sim2real_master/install/setup.bash; source /home/hightorque/wego_minipi_ws/devel/setup.bash; source /home/hightorque/realsense_ws/devel/setup.bash; export ROS_PACKAGE_PATH=/home/hightorque/sim2real_master/install/share:/home/hightorque/wego_minipi_ws/devel/share:/home/hightorque/realsense_ws/devel/share:$ROS_PACKAGE_PATH

echo '======================================'
echo '   Wego GUI User Startup Launcher     '
echo '======================================'

echo 'Waiting 50 seconds for Robot Initialization...'
for i in {50..1}; do echo -ne "Running in $i seconds... \r"; sleep 1; done
echo -e "\n[Launcher] Timer finished. Checking ROS..."

until rostopic list > /dev/null 2>&1; do echo 'Waiting for ROS Master...'; sleep 2; done
echo '[Launcher] ROS Ready! Executing enabled user scripts...'

for f in /home/hightorque/startup/custom/*.sh; do
  if [ -f "$f" ]; then echo "[Launcher] Running $f ..."; bash "$f" & fi
done

echo '[Launcher] All scripts triggered.'
wait
sleep 1000
