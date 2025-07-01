# Subsystems

- **GPS Publisher** (`gps.py`)  
  Listens to the Adafruit GPS over serial, parses NMEA, and publishes `sensor_msgs/NavSatFix` on `/gps/fix` and `gps_common/GPSFix` on `/gps/fix_common`.

- **IMU Publisher** (`imu.py`)  
  Listens to the Arduino IMU serial output and publishes it as `sensor_msgs/Imu` on `/imu/data` for downstream fusion.


As the project is building, subsytems will be added into this directory. These subsystems will also be listed here