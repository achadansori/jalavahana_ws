#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def gps_callback(data):
    """Callback function to process GPS data."""
    latitude = data.latitude
    longitude = data.longitude
    altitude = data.altitude

    rospy.loginfo(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}")

def gps_node():
    """Initialize the GPS node."""
    rospy.init_node('gps_node', anonymous=True)

    # Subscribe to the GPS topic published by MAVROS
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)

    rospy.loginfo("GPS Node has started. Listening for GPS data...")
    rospy.spin()

if __name__ == '__main__':
    try:
        gps_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("GPS Node has been terminated.")
