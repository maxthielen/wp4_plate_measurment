# WP4 Plate Measurment

## Embedded Path Planning

1. Access the UR5 pendant
2. Click on Files (top right) and exit
3. Click on Program Robot, Load Program, and select the visir_wp4_demo.urp file
4. Here the Start and Stop poseses of the robot can be altered as well as which digital output to alter to trigger the sick laser scanner.
5. Click save or save as
6. To run this program from the pendant click exit again
7. Click run program
8. Click on files (top right) and Load Program
9. Selected desired .urp file
10. Press Play (bottom) and handle pop_up

## Build Image

1. Make sure systems has **wifi connection** (able to dowload packages online)
2. Run >**./dev_build_image.sh**

## Bring Up Container v1 (scan_service)

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

## Bring Up Container v2 (embedded_scan_path)

1. Start UR5 by pressing green button

2. start the container, execute:

```console
./dev_container_bringup.sh 
```

3. Click on Docker icon on left bar within VSCode (there should be a container running in top left corner)
4. Righ click on the running container and open in new VSCode window
5. Open new instance of a terminal (inside running Docker container)
6. ?, exectute:

```console
. install/setup.bash
```

7. initialize the connection to the ur5 and dashboard client, execute: (makes services available)

```console
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.131 launch_dashboard_client:=true launch_rviz:=false
```

8. Open second terminal
9. load embedded scan path program on ur5, execute:

```console
ros2 service call /dashboard_client/load_program ur_dashboard_msgs/srv/Load filename:\ \'visir_wp4_demo.urp\'
```

10. engage the loaded program, execture:

```console
ros2 service call /dashboard_client/play std_srvs/srv/Trigger {}
```

11. run the loaded program, execute:

```console
ros2 service call /dashboard_client/close_popup std_srvs/Trigger {}
```

12. disengage the loaded program, execture:

```console
ros2 service call /dashboard_client/stop std_srvs/srv/Trigger {}
```

13. io controll, execute:

```console
python3 src/flip_master/main.py
```

## Manual UR5 Move-it

1. Run >**ros2 control switch_controllers --start joint_group_position_controller --stop joint_trajectory_controller**
2. Run >**ros2 run ur5_scan_path_service servo_keyboard_input**
