#!/usr/bin/env python3

import rospy
import serial
import json
from std_msgs.msg import Int16MultiArray

class RemoteControlNode:
    def __init__(self):
        rospy.init_node('remote_control_node', anonymous=True)
        
        # Serial setup
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.serial = None
        self.connect_serial()
        
        # Remote data array
        self.remote_data = [1500] * 10  # Initialize with center values
        
        # Publisher untuk data remote mentah
        self.raw_data_pub = rospy.Publisher('/remote/raw_data', Int16MultiArray, queue_size=10)
        
        # Rate
        self.rate = rospy.Rate(50)  # 50 Hz untuk pembacaan serial
        
    def connect_serial(self):
        """Establish serial connection"""
        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            rospy.loginfo(f"Connected to {self.serial_port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to connect to serial port: {e}")
            self.serial = None
    
    def read_serial(self):
        """Read and parse serial data from Arduino"""
        if self.serial is None or not self.serial.is_open:
            return False
            
        try:
            if self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8').strip()
                try:
                    # Assuming Arduino sends JSON formatted data
                    data = json.loads(line)
                    if isinstance(data, list) and len(data) == 10:
                        self.remote_data = data
                        return True
                except json.JSONDecodeError:
                    rospy.logwarn("Failed to parse serial data")
        except serial.SerialException as e:
            rospy.logerr(f"Serial error: {e}")
            self.serial = None
        
        return False
    
    def publish_remote_data(self):
        """Publish remote control data"""
        msg = Int16MultiArray()
        msg.data = self.remote_data
        self.raw_data_pub.publish(msg)
    
    def run(self):
        """Main run loop"""
        while not rospy.is_shutdown():
            if self.read_serial():
                self.publish_remote_data()
            self.rate.sleep()
        
        # Clean up
        if self.serial is not None and self.serial.is_open:
            self.serial.close()

if __name__ == '__main__':
    try:
        remote_node = RemoteControlNode()
        remote_node.run()
    except rospy.ROSInterruptException:
        pass