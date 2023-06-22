# WP4 Plate Measurment

## Bring Up

1. Start UR5 by pressing green button
2. Run >**./dev_container_bringup.sh**
3. Click on Docker icon on left bar within VSCode (there should be a container running in top left corner)
4. Righ click on the running container and open in new VSCode window
5. Open new instance of a terminal (inside running Docker container)
6. Run >**source install/setup.bash**
7. Run >**ros2 launch ur5_scan_path_service scan_service.launch.py**
8. Open second terminal
9. Run >**ros2 service call /dashboard_client/play std_srvs/Trigger {}** (make UR5 services available)
    1. If command return **False**: Run >**ros2 service call /dashboard_client/stop std_srvs/Trigger {}**
    2. Re-run step 9
    3. If service is **unavailable**: check that **Ethernet** is connected to UR5 and systems **wifi is turned OFF**
10. Run >**ros2 service call /trigger_movement/move std_srvs/Trigger {}** (to move to starting pos)
11. Run >**ros2 service call /trigger_movement/scan std_srvs/Trigger {}** (to move along scan path)

## Manual UR5 Move-it

1. Run >**ros2 control switch_controllers --start joint_group_position_controller --stop joint_trajectory_controller**
2. Run >**ros2 run ur5_scan_path_service servo_keyboard_input**
