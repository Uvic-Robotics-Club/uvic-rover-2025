# launch/

This directory contains ROS launch files to start and configure the rover’s subsystems.

## Contents

- **bringup.launch**  
  Master launch that (currently) brings up the full rover stack: robot description, odometry nodes, state estimation, and sensor transforms.

- **robot_description.launch**  
  Loads the URDF/Xacro description and runs `robot_state_publisher` to broadcast static transforms (`base_link` → `imu_link`, `gps_link`, etc.).

- **navsat_transform.launch**  
  Launches the `navsat_transform_node` to convert GPS fixes into local UTM coordinates and publish the `map` → `odom` transform from GPS data.

- **ekf_global.launch**  
  Starts the global `robot_localization` EKF (using `ekf_global.yaml`) to fuse GPS + IMU and publish `map` → `odom`.

- **ekf_local.launch**  
  Starts the local `robot_localization` EKF (using `ekf_local.yaml`) to fuse IMU (and optional visual/wheel odometry) and publish `odom` → `base_link`.

> **Tip:** Use  
> ```bash
> roslaunch uvic_rover bringup.launch
> ```  
> to run everything at once, or invoke individual launch files (e.g. `roslaunch uvic_rover ekf_local.launch`) for targeted testing. 