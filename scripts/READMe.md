# scripts/

This folder holds standalone Python scripts you run manually to help with the rover—such as one-off tools, utilities, or data exporters. These are **not** the main ROS nodes (those live under `nodes/`), nor Arduino firmware (which lives in `firmware/` or `arduino/`). Scripts here are installed via `catkin_install_python()` so you can call them from your workspace (e.g. `rosrun uvic_rover my_tool.py` or `python3 scripts/my_tool.py`).

## Contents

- Executable Python files you launch by hand  
- Utilities that don’t call `rospy.init_node()` or run in a spin loop  
- Helpers for tasks you perform from your shell  

---

## Examples of potential scripts

- **Calibration tools** (e.g. IMU or camera parameter sweeps)  
- **Data converters & loggers** (e.g. export bag data to CSV)  
- **Maintenance utilities** (e.g. clean build artifacts, reset configs)  

---

> **tips:**  
> - Anything that flashes or builds Arduino code belongs in `firmware/` or `arduino/`.  
> - Full ROS nodes (scripts that initialize a ROS node, subscribe/publish topics, and spin) belong in `nodes/`.  
> - Use `scripts/` for tools you run once in a while from the command line—not for the robot’s ongoing data-flow components.  
