# Subsystems

As the project is building, subsytems will be added into this directory. These subsystems will also be listed here.

# Requirements

Below are the following dependencies which these scripts in this src folder require:

sensor_msgs: Provides messages for common robotic sensors, including GPS and IMU data.
Installation command:
sudo apt install ros-noetic-sensor-msgs

gps_common: Provides GPS-related message types, including GPSFix.
Installation command:
sudo apt install ros-noetic-gps-common

robot_localization: ROS package that provides nonlinear state estimation through sensor fusion of IMU, GPS, wheel odometry, and other localization data using Extended and Unscented Kalman Filters.
Installation command:
sudo apt install ros-noetic-robot-localization

adafruit-circuitpython-gps: Adafruit's CircuitPython library for GPS modules.
Installation command:
pip install adafruit-circuitpython-gps