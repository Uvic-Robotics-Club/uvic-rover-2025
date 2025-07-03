# config/

This directory contains YAML configuration files used to parameterize ROS nodes and launch settings across the rover’s software stack.

## Contents

- **ekf_global.yaml**: Parameters for the global EKF (GPS + IMU) that publishes the `map` → `odom` transform.  
- **ekf_local.yaml**: Parameters for the local EKF (IMU and optional visual/wheel odometry) that publishes the `odom` → `base_link` transform.  