# urdf/

This folder holds the robot’s Xacro/URDF description files, defining the chassis and sensor mounting geometry for the rover.

## Contents

- **robot.urdf.xacro**  
  A parameterized Xacro file that describes:
  - `base_link` — the main body of the rover  
  - `imu_link` & `gps_link` — sensor frames  
  - Fixed joints with adjustable offset properties (`imu_offset_*`, `gps_offset_*`)