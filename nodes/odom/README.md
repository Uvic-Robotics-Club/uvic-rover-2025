# nodes/odom/

This directory contains the ROS node entry-points responsible for providing raw odometry data. Each script implements a standalone node that publishes sensor data under its own topic and frame.

## gps.py

- **Node name:** `gps_talker`  
- **Purpose:**  
  - Detects the USB GPS module and configures it via PMTK commands  
  - Reads NMEA sentences at 1 Hz  
  - Publishes  
    - `/gps/fix` (`sensor_msgs/NavSatFix`)  
    - `/gps/fix_common` (`gps_common/GPSFix`)  
  - Uses the `gps_link` frame  

## imu.py

- **Node name:** `imu_talker`  
- **Purpose:**  
  - Opens the serial connection to the IMU Arduino  
  - Parses fused or raw IMU samples at ~50 Hz  
  - Publishes `/imu/data` (`sensor_msgs/Imu`) with orientation, acceleration, angular velocity, and covariance fields populated  
  - Uses the `imu_link` frame  

## Extending odometry

To add more odometry sources (wheel encoders, LiDAR odometry, etc.), place their node scripts here so all odometry-related entry points remain grouped. 