# nodes/

This folder contains your ROS node entry-points: executable Python scripts that call `rospy.init_node()`, create publishers/subscribers, and spin.  

## Contents

- **Node scripts** (e.g. `gps.py`, `imu.py`, `aruco_detector.py`)  
- Each file should:
  1. Start with a she-bang, e.g. `#!/usr/bin/env python3`  
     This line tells the operating system which Python interpreter to invoke when you run the script directly, ensuring it uses the correct environment. 
     It guarantees your nodes “just work” when you, for instance, rosrun uvic_rover gps.py (or any direct call), without making users remember to 
     prepend python3.  
  2. Be made executable (`chmod +x`)  
  3. Be installed via `catkin_install_python()` in `CMakeLists.txt`, so you can `rosrun uvic_rover gps.py`