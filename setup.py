#!/usr/bin/env python3

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['uvic_rover', 'uvic_rover.odom'],
    package_dir={'': 'src'}
)

setup(**setup_args)
