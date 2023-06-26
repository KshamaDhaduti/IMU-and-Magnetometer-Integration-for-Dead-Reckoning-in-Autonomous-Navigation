#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#---IMU Noise Characterization with Allan Variance- EECE 55554 - Assignment 3---
#---Created by Kshama Dhaduti---

from ast import Bytes
from mmap import PROT_WRITE
import rospy
import serial
import time
import sensor_msgs
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header

if __name__ == '__main__':
    SENSOR_NAME = "imu_sensor"
    
    # Create publishers for IMU and MagneticField messages
    pub1 = rospy.Publisher("imu", Imu, queue_size=10)
    pub2 = rospy.Publisher("mag", MagneticField, queue_size=10)
    
    # Initialize the ROS node
    rospy.init_node('gps_sensor')
    
    # Get serial port, baudrate, and sampling rate parameters from ROS parameter server
    serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate', 115200)
    sampling_rate = rospy.get_param('~sampling_rate', 40.0)
   
    # Initialize the serial port for communication with the IMU sensor
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    
    # Log debug information
    rospy.logdebug("Using imu sensor on port " + serial_port + " at " + str(serial_baud))
    rospy.logdebug("Initializing sensor with *0100P4\\r\\n ...")
    
    # Calculate the number of samples to be taken based on the sampling rate
    sampling_count = int(round(1 / (sampling_rate * 0.007913)))
    
    # Sleep for a short duration to allow the sensor to initialize
    rospy.sleep(0.2)   
    
    # Send a command to the sensor for initialization
    port.write(b'$VNWRG,07,40*XX')     
    
    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing IMU and Magnetometer data")
            
    # Create header objects for IMU and MagneticField messages
    h1 = Header()
    h2 = Header()
    
    # Create instances of the Imu and MagneticField messages
    msg_imu = Imu()
    msg_mag = MagneticField()
    
    # Initialize sequence number
    i = 1

    try:
        while not rospy.is_shutdown():
            # Read a line of data from the serial port
            line = port.readline()
            
            if line == 'no data ':
                # If no data is received, display a warning
                rospy.logwarn("VNYMR: No data")
            else:
                if line.startswith(b'$VNYMR'):
                    # If the line starts with "$VNYMR", it contains IMU data
                    
                    # Split the line into comma-separated values
                    s = line.split(b",")
                    
                    # Extract yaw, pitch, and roll values from the data
                    yaw = s[1].decode('utf-8')
                    pitch = s[2].decode('utf-8')
                    roll = s[3].decode('utf-8')
                    
                    # Extract magnetometer values from the data
                    magx = s[4].decode('utf-8')
                    magy = s[5].decode('utf-8')
                    magz = s[6].decode('utf-8')
                    
                    # Extract accelerometer values from the data
                    aclx = s[7].decode('utf-8')
                    acly = s[8].decode('utf-8')
                    aclz = s[9].decode('utf-8')
                    
                    # Extract gyroscope values from the data
                    gyrx = s[10].decode('utf-8')
                    gyry = s[11].decode('utf-8')
                    gyrz = s[12].decode('utf-8')
                    gyrz = gyrz[:-5]
                    
                    # Create a Rotation object from the Euler angles (roll, pitch, yaw)
                    rot = Rotation.from_euler('xyz', [float(roll), float(pitch), float(yaw)], degrees=True)
                    
                    # Convert the rotation to quaternion representation
                    quat = rot.as_quat()
                    
                    # Extract quaternion components
                    x = quat[0]
                    y = quat[1]
                    z = quat[2]
                    w = quat[3]
                    
                    # Print the extracted values for debugging purposes
                    print("Yaw: " + yaw + " Pitch: " + pitch + " Roll: " + roll)
                    print("Magx: " + magx + " Magy: " + magy + " Magz: " + magz)
                    print("Aclx: " + aclx + " Acly: " + acly + " Aclz: " + aclz)
                    print("Gyrx: " + gyrx + " Gyry: " + gyry + " Gyrz: " + gyrz)
                    print("x: " + str(x) + " y: " + str(y) + " z: " + str(z) + " w: " + str(w))
    
                    # Fill in the header and message fields with the extracted data
                    h1.seq = i
                    h1.stamp = rospy.get_rostime()
                    h1.frame_id = "IMU DATA"
                    h2.stamp = rospy.get_rostime()
                    h2.frame_id = "MAG DATA"
                    msg_imu.header = h1
                    msg_mag.header = h2
                    msg_imu.orientation.x = float(x)
                    msg_imu.orientation.y = float(y)
                    msg_imu.orientation.z = float(z)
                    msg_imu.orientation.w = float(w)
                    msg_imu.angular_velocity.x = float(gyrx)
                    msg_imu.angular_velocity.y = float(gyry)
                    msg_imu.angular_velocity.z = float(gyrz)
                    msg_imu.linear_acceleration.x = float(aclx)
                    msg_imu.linear_acceleration.y = float(acly)
                    msg_imu.linear_acceleration.z = float(aclz)
                    msg_mag.magnetic_field.x = float(magx)
                    msg_mag.magnetic_field.y = float(magy)
                    msg_mag.magnetic_field.z = float(magz)
                    
                    # Publish the IMU and MagneticField messages
                    pub1.publish(msg_imu)
                    pub2.publish(msg_mag)
                    
                    # Increment the sequence number
                    i = i + 1
    except rospy.ROSInterruptException:
        port.close()
