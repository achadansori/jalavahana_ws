#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import String, Bool, Int16

def parse_serial_data(serial_line):
    """
    Parse the serial data line into an array of integers.
    """
    try:
        data = list(map(int, serial_line.strip().split()))
        return data
    except ValueError:
        rospy.logwarn("Failed to parse serial data")
        return None

def process_remote_data(data):
    """
    Process the parsed serial data and return the values for mode, arming, throttle, and yaw.
    """
    if len(data) < 8:  # Ensure data has enough elements
        rospy.logwarn("Incomplete data received")
        return None, None, None, None

    # Process mode
    if data[6] < 1300:
        mode = "manual"
    elif data[6] > 1700:
        mode = "auto"
    else:
        mode = "neutral"

    # Process arming
    arming = data[7] > 1500

    # Throttle and yaw
    throttle = data[2]
    yaw = data[0]

    return mode, arming, throttle, yaw

def remote_node():
    """
    Remote Node for reading serial data and publishing remote control commands.
    """
    rospy.init_node('remote_node', anonymous=True)

    # Publishers for the topics
    mode_pub = rospy.Publisher('mode', String, queue_size=10)
    arming_pub = rospy.Publisher('arming', Bool, queue_size=10)
    throttle_pub = rospy.Publisher('throttle_remote', Int16, queue_size=10)
    yaw_pub = rospy.Publisher('yaw_remote', Int16, queue_size=10)

    # Initialize serial connection
    serial_port = "/dev/ttyACM0"
    baud_rate = 9600
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        rospy.loginfo(f"Connected to serial port: {serial_port} at {baud_rate} baud")
    except serial.SerialException as e:
        rospy.logerr(f"Serial port error: {e}")
        return

    # Loop to read and process serial data
    rate = rospy.Rate(50)  # 50 Hz
    while not rospy.is_shutdown():
        try:
            if ser.in_waiting > 0:
                serial_line = ser.readline().decode('utf-8')
                data = parse_serial_data(serial_line)
                if data:
                    mode, arming, throttle, yaw = process_remote_data(data)
                    if mode is not None:
                        mode_pub.publish(mode)
                        arming_pub.publish(arming)
                        throttle_pub.publish(throttle)
                        yaw_pub.publish(yaw)
        except Exception as e:
            rospy.logerr(f"Error reading serial data: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        remote_node()
    except rospy.ROSInterruptException:
        pass
