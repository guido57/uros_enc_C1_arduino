# OVERVIEW

This is a micro-ROS ESP32 program to:

* subscribe to cmd_vel
* drive two motors (DiffDrive) with PID, using thier wheel encoders
* read the lidar SLAMTEC C1 and publish /scan
* publish /odom and the odom to base_link transformation on /tf
* publish /wifi with SSID, BSSID and RSSI
* publish /imu
* publish /battery_state: voltage and current


# SCHEMATIC

![](docs/schematic.png)


  
