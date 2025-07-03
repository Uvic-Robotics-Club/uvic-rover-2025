# src/

This directory defines your importable Python package namespace. Everything here is installed onto `PYTHONPATH` via `setup.py` (using `catkin_python_setup()`).

## Contents

- **`<package_name>/`** (e.g. `uvic_rover/`):  
  - `__init__.py` — marks the package  
  - Sub-packages (e.g. `odom/`, `vision/`) with reusable libraries and helpers  
- **`tests/`** (optional): unit tests for package code

## Usage

Import shared modules in your nodes and utilities, for example:
```python
from uvic_rover.odom.filter import EKFWrapper
```

**Tip:** This folder is for importable libraries. For standalone scripts and one-off tools, use the scripts/ directory.