#!/usr/bin/env python3
import time
import adafruit_gps
import rospy
from sensor_msgs.msg import NavSatFix
from gps_common.msg import GPSFix
import serial
import serial.tools.list_ports
# Create a serial connection for the GPS connection using default speed and
# a slightly higher timeout (GPS modules typically update once a second).
# These are the defaults you should use for the GPS FeatherWing.
# For other boards set RX = GPS module TX, and TX = GPS module RX pins.
#uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=10)

# for a computer, use the pyserial library for uart access

def find_gps_port():
    # Iterate over all available serial ports
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        #print(f"DEBUG: Device: {port.device}, VID: {port.vid}, PID: {port.pid}, Description: {port.description}")
        # Check if the port has the desired vendor and product ID
        if port.vid == 0x10c4 and port.pid == 0xea60: # If correct GPS
            rospy.loginfo(f"GPS device found on {port.device}")
            return serial.Serial(port.device, baudrate=9600, timeout=10)      
    rospy.logfatal("GPS device not found!")
    rospy.signal_shutdown("GPS device not found!")
    raise rospy.ROSInterruptException("GPS device not found!")

def main():
    # Initialize GPS Ros node 
    rospy.init_node('gps_talker')

    navsatfix_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)    
    navsatfix_msg = NavSatFix()
    navsatfix_msg.header.frame_id = "gps_link"
    navsatfix_msg.header.stamp = rospy.Time.now()

    common_pub = rospy.Publisher('/gps/fix_common', GPSFix, queue_size=10) 
    common_msg = GPSFix()
    common_msg.header.frame_id = "gps_link"
    common_msg.header.stamp = rospy.Time.now()

    rate = rospy.Rate(1)

    uart = find_gps_port()
    # Create a GPS module instance.
    gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial
    # gps = adafruit_gps.GPS_GtopI2C(i2c, debug=False)  # Use I2C interface

    # Initialize the GPS module by changing what data it sends and at what rate.
    # These are NMEA extensions for PMTK_314_SET_NMEA_OUTPUT and
    # PMTK_220_SET_NMEA_UPDATERATE but you can send anything from here to adjust
    # the GPS module behavior:
    #   https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf

    # Turn on the basic GGA and RMC info (what you typically want)
    gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
    # Turn on just minimum info (RMC only, location):
    # gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
    # Turn off everything:
    # gps.send_command(b'PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
    # Turn on everything (not all of it is parsed!)
    # gps.send_command(b'PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0')

    # Set update rate to once a second (1hz) which is what you typically want.
    gps.send_command(b"PMTK220,1000")
    # Or decrease to once every two seconds by doubling the millisecond value.
    # Be sure to also increase your UART timeout above!
    # gps.send_command(b'PMTK220,2000')
    # You can also speed up the rate, but don't go too fast or else you can lose
    # data during parsing.  This would be twice a second (2hz, 500ms delay):
    # gps.send_command(b'PMTK220,500')

    # Main loop runs forever printing the location, etc. every second.
    last_print = time.monotonic()
    while not rospy.is_shutdown():
        # Make sure to call gps.update() every loop iteration and at least twice
        # as fast as data comes from the GPS unit (usually every second).
        # This returns a bool that's true if it parsed new data (you can ignore it
        # though if you don't care and instead look at the has_fix property).
        gps.update()
        # Every second print out current location details if there's a fix.
        current = time.monotonic()
        if current - last_print >= 1.0:
            last_print = current
            if not gps.has_fix:
                # Try again if we don't have a fix yet.
                rospy.logdebug("Waiting for fix...")
                continue
            # We have a fix! (gps.has_fix is true)
            
            # Put gps data into navstat
            navsatfix_msg.latitude = gps.latitude
            navsatfix_msg.longitude = gps.longitude
            navsatfix_msg.altitude = gps.altitude_m
            navsatfix_msg.status.status = 1
            navsatfix_msg.status.service = 1

            common_msg.latitude = gps.latitude
            common_msg.longitude = gps.longitude
            common_msg.altitude = gps.altitude_m
            common_msg.status.status = 1

            navsatfix_pub.publish(navsatfix_msg)
            common_pub.publish(common_msg)

            # Print out details about the fix like location, date, etc.
            rospy.logdebug("=" * 40)  # Print a separator line.           
            rospy.logdebug(
            "Fix UTC timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(
            gps.timestamp_utc.tm_mon,  # Grab parts of the time from the
            gps.timestamp_utc.tm_mday,  # struct_time object that holds
            gps.timestamp_utc.tm_year,  # the fix time.  Note you might
            gps.timestamp_utc.tm_hour,  # not get all data like year, day,
            gps.timestamp_utc.tm_min,  # month!
            gps.timestamp_utc.tm_sec,
                )
            )
            rospy.logdebug("Latitude: {0:.6f} degrees".format(gps.latitude))
            rospy.logdebug("Longitude: {0:.6f} degrees".format(gps.longitude))
            rospy.logdebug(
                "Precise Latitude: {:2.0f}{:2.4f} degrees".format(
                    gps.latitude_degrees, gps.latitude_minutes
                )
            )
            rospy.logdebug(
                "Precise Longitude: {:2.0f}{:2.4f} degrees".format(
                    gps.longitude_degrees, gps.longitude_minutes
                )
            )
            # print("Fix quality: {}".format(gps.fix_quality))
            # # Some attributes beyond latitude, longitude and timestamp are optional
            # # and might not be present.  Check if they're None before trying to use!
            # if gps.satellites is not None:
            #     print("# satellites: {}".format(gps.satellites))
            # if gps.altitude_m is not None:
            #     print("Altitude: {} meters".format(gps.altitude_m))
            # if gps.speed_knots is not None:
            #     print("Speed: {} knots".format(gps.speed_knots))
            # if gps.track_angle_deg is not None:
            #     print("Track angle: {} degrees".format(gps.track_angle_deg))
            # if gps.horizontal_dilution is not None:
            #     print("Horizontal dilution: {}".format(gps.horizontal_dilution))
            # if gps.height_geoid is not None:
            #     print("Height geoid: {} meters".format(gps.height_geoid))

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
