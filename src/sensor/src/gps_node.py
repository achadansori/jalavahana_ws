#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from math import *

class GPSNode:
    def __init__(self):
        rospy.init_node('gps_node', anonymous=True)
        
        # Subscriber untuk data GPS dari MAVROS
        self.gps_sub = rospy.Subscriber('/mavros/global_position/global', 
                                      NavSatFix, 
                                      self.gps_callback)
        
        # State subscriber
        self.state_sub = rospy.Subscriber('/mavros/state', 
                                        State, 
                                        self.state_callback)
        
        # Publisher untuk posisi GPS
        self.pos_pub = rospy.Publisher('/usv/gps_position', 
                                     NavSatFix, 
                                     queue_size=10)
        
        self.current_state = State()
        self.current_position = NavSatFix()
        
        # Rate
        self.rate = rospy.Rate(10)  # 10 Hz
        
    def state_callback(self, msg):
        self.current_state = msg
        
    def gps_callback(self, msg):
        self.current_position = msg
        
        # Log posisi GPS
        rospy.loginfo("Latitude: {:.7f}, Longitude: {:.7f}, Altitude: {:.2f}"
                     .format(msg.latitude, msg.longitude, msg.altitude))
        
        # Publish data GPS
        self.pos_pub.publish(msg)
        
    def run(self):
        while not rospy.is_shutdown():
            if self.current_state.connected:
                rospy.loginfo_throttle(5, "Connected to FCU")
            else:
                rospy.logwarn_throttle(5, "Not connected to FCU")
                
            self.rate.sleep()

if __name__ == '__main__':
    try:
        gps_node = GPSNode()
        gps_node.run()
    except rospy.ROSInterruptException:
        pass