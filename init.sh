#! /bin/bash
echo "Starting initialization procedure inside plate measurment container"

source install/setup.bash
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.131 launch_dashboard_client:=true launch_rviz:=false &
python3 src/flip_master/main.py 