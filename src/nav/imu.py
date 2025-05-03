#!/usr/bin/env python3
import math
import serial
import serial.tools.list_ports
import rospy
from sensor_msgs.msg import Imu
import tf.transformations as tft
from dataclasses import dataclass

# Conversion factors
G_TO_MS2 = 9.81
DEG_TO_RAD = math.pi / 180

@dataclass
class ImuData:
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    mx: float = 0.0
    my: float = 0.0
    mz: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    qw: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0

def shutdown_hook(ser_port):
    rospy.loginfo("Shutting down IMU node...")
    ser_port.close()

def find_imu_port():
    # Iterate over all available serial ports
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        # print(f"DEBUG: Device: {port.device}, VID: {port.vid}, PID: {port.pid}, Description: {port.description}")
        # Check if the port has the desired vendor and product ID
        if port.vid == 0x2341 and port.pid == 0x0043: # If a MPU9250 IMU (detects the arduino it's connected to)
            print(f"IMU device found on {port.device}")
            return serial.Serial(port.device, baudrate=115200, timeout=0.1)
    rospy.logfatal("IMU device not found!")
    rospy.signal_shutdown("IMU device not found!")
    raise rospy.ROSInterruptException("IMU device not found!")

def parse_serial_data(ser_port):
    """
    Reads and parses the serial string into 9 float values.
    Returns a tuple of floats if successful, otherwise None.
    """
    try:
        raw_ser = ser_port.readline().decode().strip()
        if not raw_ser:
            return None

        parts = raw_ser.split()
        if len(parts) != 13: # checks for x pieces of information
            rospy.logwarn("Unexpected data length: %d", len(parts))
            return None
        return tuple(map(float, parts))
    except ValueError as e:
        rospy.logwarn("Error parsing data: %s", e)
        return None

# def create_quarternion(imu_data):
#     """
#     Computes roll, pitch, and yaw from accelerometer and magnetometer values,
#     then converts these angles into a quaternion.
#     Returns a tuple (qw, qx, qy, qz).
#     """
#     roll = math.atan2(imu_data.ay, imu_data.az)
#     pitch = math.atan2(-imu_data.ax, math.sqrt(imu_data.ay**2 + imu_data.az**2))
    
#     mx_comp = imu_data.mx * math.cos(pitch) + imu_data.mz * math.sin(pitch)
#     my_comp = (imu_data.mx * math.sin(roll) * math.sin(pitch) +
#                imu_data.my * math.cos(roll) -
#                imu_data.mz * math.sin(roll) * math.cos(pitch))
#     yaw = math.atan2(-my_comp, mx_comp)
    
#     return tuple(tft.quaternion_from_euler(roll, pitch, yaw))

def create_imu_msg(imu_data):
    """
    Creates and returns an Imu message populated with sensor values from imu_data.
    """
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = "imu_link"

    imu_msg.orientation.x = imu_data.qx
    imu_msg.orientation.y = imu_data.qy
    imu_msg.orientation.z = imu_data.qz
    imu_msg.orientation.w = imu_data.qw

    imu_msg.linear_acceleration.x = imu_data.ax
    imu_msg.linear_acceleration.y = imu_data.ay
    imu_msg.linear_acceleration.z = imu_data.az

    imu_msg.angular_velocity.x = imu_data.gx
    imu_msg.angular_velocity.y = imu_data.gy
    imu_msg.angular_velocity.z = imu_data.gz

    return imu_msg

def print_log(imu_data):
    divider = "-" * 60
    rospy.loginfo(
        f"\n{divider}\n"
        f"Accelerometer (m/s²): X = {imu_data.ax:.3f}, Y = {imu_data.ay:.3f}, Z = {imu_data.az:.3f}\n"
        f"Gyroscope (rad/s):    X = {imu_data.gx:.3f}, Y = {imu_data.gy:.3f}, Z = {imu_data.gz:.3f}\n"
        f"Quaternion:           W = {imu_data.qw:.3f}, X = {imu_data.qx:.3f}, "
        f"Y = {imu_data.qy:.3f}, Z = {imu_data.qz:.3f}\n"
        f"{divider}"
    )

def main():
    rospy.init_node('imu_talker')
    imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    rate = rospy.Rate(30)

    # Open and configure the serial port.
    ser_port = find_imu_port()
    ser_port.reset_input_buffer()
    rospy.on_shutdown(lambda: shutdown_hook(ser_port))

    while not rospy.is_shutdown():
        try:
            parsed_ser = parse_serial_data(ser_port)
            imu_data = ImuData()
            if parsed_ser is None:
                continue
              
            ax_raw, \
            ay_raw, \
            az_raw, \
            imu_data.mx, \
            imu_data.my, \
            imu_data.mz, \
            gx_raw, \
            gy_raw, \
            gz_raw, \
            imu_data.qw, \
            imu_data.qx, \
            imu_data.qy, \
            imu_data.qz = parsed_ser 

            imu_data.ax = ax_raw * G_TO_MS2
            imu_data.ay = ay_raw * G_TO_MS2
            imu_data.az = az_raw * G_TO_MS2
            imu_data.gx = gx_raw * DEG_TO_RAD
            imu_data.gy = gy_raw * DEG_TO_RAD
            imu_data.gz = gz_raw * DEG_TO_RAD

            imu_pub.publish(create_imu_msg(imu_data))
            print_log(imu_data)
            rate.sleep()

        except serial.SerialException as e:
            rospy.logerr("Serial port error: %s", e)
        except Exception as e:
            rospy.logerr("Unexpected error: %s", e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass