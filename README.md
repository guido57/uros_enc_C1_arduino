# OVERVIEW

This is a micro-ROS ESP32 program to:

* subscribe to cmd_vel
* drive two motors (DiffDrive) with PID, using the wheel encoders
* read the lidar LDROBOT C1 and publish /scan
* publish /odom and the odom to base_link transformation on /tf

  
