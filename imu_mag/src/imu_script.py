#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#---Navigation with IMU and Magnetometer - EECE5554 - Assignment 4 ---
#---Created by Kshama Dhaduti---

from ast import Bytes  # Importing Bytes from ast library (unused)
from mmap import PROT_WRITE  # Importing PROT_WRITE from mmap library (unused)
import rospy  # Importing the rospy library for ROS
import serial  # Importing the serial library for serial communication
import time  # Importing the time library for delays
import sensor_msgs  # Importing sensor_msgs package
from sensor_msgs.msg import Imu, MagneticField  # Importing Imu and MagneticField messages from sensor_msgs
from scipy.spatial.transform import Rotation  # Importing Rotation from scipy.spatial.transform
from std_msgs.msg import Header  # Importing Header message from std_msgs

if __name__ == '__main__':
    SENSOR_NAME = "imu_sensor"
    pub1 = rospy.Publisher("imu", Imu, queue_size=10)  # Create publisher for IMU messages
    pub2 = rospy.Publisher("mag", MagneticField, queue_size=10)  # Create publisher for MagneticField messages
    rospy.init_node('gps_sensor')  # Initialize the ROS node with the name 'gps_sensor'
    serial_port = rospy.get_param('~port', '/dev/ttyUSB1')  # Get the serial port parameter from the ROS parameter server
    serial_baud = rospy.get_param('~baudrate', 115200)  # Get the baudrate parameter from the ROS parameter server
    sampling_rate = rospy.get_param('~sampling_rate', 40.0)  # Get the sampling rate parameter from the ROS parameter server
   
    port = serial.Serial(serial_port, serial_baud, timeout=3.)  # Initialize the serial port for communication with the IMU sensor
    rospy.logdebug("Using imu sensor on port " + serial_port + " at " + str(serial_baud))  # Log debug information
    rospy.logdebug("Initializing sensor with *0100P4\\r\\n ...")
    sampling_count = int(round(1 / (sampling_rate * 0.007913)))  # Calculate the number of samples to be taken based on the sampling rate
    rospy.sleep(0.2)  # Sleep for a short duration to allow the sensor to initialize
    port.write(b'$VNWRG,07,40*XX')  # Send a command to the sensor for initialization
    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing IMU and Magnetometer data")
            
    h1 = Header()  # Create a Header object for IMU messages
    msg_imu = Imu()  # Create an instance of the Imu message
    msg_mag = MagneticField()  # Create an instance of the MagneticField message
    
    i = 1  # Initialize sequence number

    try:
        while not rospy.is_shutdown():
            line = port.readline()  # Read a line of data from the serial port
            if line == 'no data ':
                rospy.logwarn("VNYMR: No data")  # If no data is received, display a warning
            else:
                if line.startswith(b'$VNYMR'):  # If the line starts with "$VNYMR", it contains IMU data
                    print(line)  # Print the received line for debugging purposes
                    s = line.split(b",")  # Split the line into comma-separated values
                    
                    # Extract yaw, pitch, and roll values from the data
                    yaw = s[1].decode('utf-8')
                    pitch = s[2].decode('utf-8')
                    roll = s[3].decode('utf-8')
                    print("Yaw: " + yaw + " Pitch: " + pitch + " Roll: " + roll)
                    
                    # Extract magnetometer values from the data
                    magx = s[4].decode('utf-8')
                    magy = s[5].decode('utf-8')
                    magz = s[6].decode('utf-8')
                    print("Magx: " + magx + " Magy: " + magy + " Magz: " + magz)
                    
                    # Extract accelerometer values from the data
                    aclx = s[7].decode('utf-8')
                    acly = s[8].decode('utf-8')
                    aclz = s[9].decode('utf-8')
                    print("Aclx: " + aclx + " Acly: " + acly + " Aclz: " + aclz)
                    
                    # Extract gyroscope values from the data
                    gyrx = s[10].decode('utf-8')
                    gyry = s[11].decode('utf-8')
                    gyrz = s[12].decode('utf-8')
                    gyrz = gyrz[:-5]  # Remove the last 5 characters
                    
                    print("Gyrx: " + gyrx + " Gyry: " + gyry + " Gyrz: " + gyrz)
                    
                    # Create a Rotation object from the Euler angles (roll, pitch, yaw)
                    rot = Rotation.from_euler('xyz', [float(roll), float(pitch), float(yaw)], degrees=True)
                    
                    # Convert the rotation to quaternion representation
                    quat = rot.as_quat()
                    
                    # Extract quaternion components
                    x = quat[0]
                    y = quat[1]
                    z = quat[2]
                    w = quat[3]
                    
                    print("x: " + str(x) + " y: " + str(y) + " z: " + str(z) + " w: " + str(w))

                    # Fill in the header and message fields with the extracted data
                    h1.seq = i
                    h1.stamp = rospy.get_rostime()
                    h1.frame_id = "IMU DATA"
                    msg_imu.header = h1
                    msg_mag.header = h1
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
