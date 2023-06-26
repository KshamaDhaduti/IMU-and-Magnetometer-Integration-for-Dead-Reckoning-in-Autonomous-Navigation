#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#---Assignment 4 EECE 5554---
#---Created by Kshama Dhaduti---

import rospy  # Importing the rospy library for ROS
import serial  # Importing the serial library for serial communication
from math import sin, pi  # Importing sin and pi functions from math
from std_msgs.msg import Float64  # Importing the Float64 message from std_msgs
import time  # Importing the time library for delays
import utm  # Importing the utm library for UTM coordinate conversions

from imu_mag.msg import custom  # Importing the custom message from the imu_mag package

if __name__ == '__main__':
    SENSOR_NAME = "gps_sensor"
    pub = rospy.Publisher("custom_message", custom, queue_size=10)  # Create publisher for custom messages
    rospy.init_node('gps_sensor')  # Initialize the ROS node with the name 'gps_sensor'
    serial_port = rospy.get_param('~port', '/dev/ttyUSB0')  # Get the serial port parameter from the ROS parameter server
    serial_baud = rospy.get_param('~baudrate', 4800)  # Get the baudrate parameter from the ROS parameter server
    sampling_rate = rospy.get_param('~sampling_rate', 5.0)  # Get the sampling rate parameter from the ROS parameter server
     
    port = serial.Serial(serial_port, serial_baud, timeout=3.)  # Initialize the serial port for communication with the GPS sensor
    rospy.logdebug("Using gps sensor on port " + serial_port + " at " + str(serial_baud))  # Log debug information
    rospy.logdebug("Initializing sensor with *0100P4\\r\\n ...")
    
    sampling_count = int(round(1 / (sampling_rate * 0.007913)))  # Calculate the number of samples to be taken based on the sampling rate
    rospy.sleep(0.2)
    
    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing longitude and latitude.")
        
    msg = custom()  # Create an instance of the custom message
    i = 1  # Initialize sequence number

    try:
        while not rospy.is_shutdown():
            msg.header.seq = i  # Set the sequence number in the custom message
            line = port.readline()  # Read a line of data from the serial port
            line2 = line.decode('latin-1')
            
            if line == '':
                rospy.logwarn("DEPTH: No data")  # If no data is received, display a warning
            else:
                if line2.startswith("$GPGGA"):  # If the line starts with "$GPGGA", it contains GPS data
                    s = line2.split(",")  # Split the line into comma-separated values
                    
                    lat = s[2]  # Get the latitude value from the data
                    lon = s[4]  # Get the longitude value from the data
                    lat_dir = s[3]  # Get the latitude direction (N/S) from the data
                    lon_dir = s[5]  # Get the longitude direction (E/W) from the data
                    utc_time = s[1]  # Get the UTC time from the data
                    alt = s[9]  # Get the altitude value from the data
                    
                    degrees_lat = int(float(lat) / 100)  # Calculate the degrees part of latitude
                    minutes_lat = float(lat) - (degrees_lat * 100)  # Calculate the minutes part of latitude
                    degrees_lon = int(float(lon) / 100)  # Calculate the degrees part of longitude
                    minutes_lon = float(lon) - (degrees_lon * 100)  # Calculate the minutes part of longitude
                    
                    dd_lat = float(degrees_lat) + float(minutes_lat) / 60  # Convert latitude to decimal degrees
                    dd_lon = float(degrees_lon) + float(minutes_lon) / 60  # Convert longitude to decimal degrees
                    
                    if lon_dir == 'W':  # If the longitude direction is west, multiply by -1
                        dd_lon *= -1
                    if lat_dir == 'S':  # If the latitude direction is south, multiply by -1
                        dd_lat *= -1

                    print("\n" + str(dd_lat) + " " + str(dd_lon))  # Print the latitude and longitude values

                    utm_data3 = utm.from_latlon(dd_lat, dd_lon)  # Convert latitude and longitude to UTM coordinates
                    
                    msg.header.stamp = rospy.get_rostime()  # Set the timestamp in the custom message header
                    msg.header.frame_id = "GPS_Data"  # Set the frame ID in the custom message header
                    msg.latitude = dd_lat  # Set the latitude value in the custom message
                    msg.longitude = dd_lon  # Set the longitude value in the custom message
                    msg.altitude = float(alt)  # Set the altitude value in the custom message
                    msg.utm_easting = utm_data3[0]  # Set the UTM easting value in the custom message
                    msg.utm_northing = utm_data3[1]  # Set the UTM northing value in the custom message
                    msg.zone = float(utm_data3[2])  # Set the UTM zone value in the custom message
                    msg.letter_field = utm_data3[3]  # Set the UTM letter field value in the custom message
                    
                    rospy.loginfo(msg)  # Log the custom message
                    pub.publish(msg)  # Publish the custom message
                    
            i += 1  # Increment the sequence number
    except rospy.ROSInterruptException:
        port.close()
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down paro_depth node...")
