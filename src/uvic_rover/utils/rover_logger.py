#!/usr/bin/env python3

import rospy
import logging
import os

from typing import Optional
from datetime import datetime

class ROSLogHandler(logging.Handler):
    """Redirects Python logging to rospy log system"""

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
            if (record.levelno == logging.DEBUG):
                rospy.logdebug(msg)
            elif (record.levelno == logging.INFO):
                rospy.loginfo(msg)
            elif (record.levelno == logging.WARNING):
                rospy.logwarn(msg)
            elif (record.levelno == logging.ERROR):
                rospy.logerr(msg)
            elif (record.levelno == logging.CRITICAL):
                rospy.logfatal(msg)
        except (rospy.ROSException, AttributeError):
            pass # rospy not initialized yet

def setup_logger(
        name: str = 'uvic_rover', # Name of the Logger (Default uvic_rover) 
        log_to_file: bool = False, # Whether or not the log should be outputted to a file (Default False)
        log_level=logging.INFO, # Default log level is INFO unless otherwise specified
        repo_root_relative: Optional[str] = "../../logs" # Essentially the path where any log files should be saved
    ) -> logging.Logger:
    """
    Set up a logger for consisten terminal and ROS-integrated logging (logs appear in rqt_console, /rosout, etc.)
    Three types of handlers - Console, File, ROS Log
    """

    logger = logging.getLogger(name)

    if (logger.hasHandlers):
        return logger # prevent duplicate handlers if called multiple 
    
    logger.setLevel(log_level)

    # How the log is formatted
    formatter = logging.Formatter('[%(asctime)s] [%(levelname)s] %(message)s',
                                  datefmt='%H:%M:%S')

    # Console (terminal) handler - logs appear on the terminal
    ch = logging.StreamHandler()
    ch.setFormatter(formatter)
    logger.addHandler(ch)

    # Optional file handler
    if log_to_file:
        logs_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), repo_root_relative))
        os.makedirs(logs_dir, exist_ok=True) # Don't overwrite existing directory
        log_filename = datetime.now().strftime("rover_log_%Y-%m-%d_%H-%M-%S.log")
        fh = logging.FileHandler(os.path.join(logs_dir, log_filename))
        fh.setFormatter(formatter)
        logger.addHandler(fh)

    # ROS log handler
    ros_handler = ROSLogHandler()
    ros_handler.setFormatter(formatter)
    logger.addHandler(ros_handler)

    return logger
