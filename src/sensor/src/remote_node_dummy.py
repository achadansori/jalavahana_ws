#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import String, Bool, Int16

def generate_dummy_data():
    """
    Generate dummy data simulating remote control input.
    Returns a list of integers in the expected format.
    """
    return [
        random.randint(1000, 2000),  # Yaw
        random.randint(1000, 2000),  # Pitch
        random.randint(1000, 2000),  # Throttle
        random.randint(1000, 2000),  # Roll
        random.randint(1000, 2000),  # Aux1
        random.randint(1000, 2000),  # Aux2
        1000,                        # Mode
        2000  # Arming
    ]

def process_remote_data(data):
    """
    Process the dummy data and return the values for mode, arming, throttle, and yaw.
    """
    if len(data) < 8:  # Ensure data has enough elements
        rospy.logwarn("Incomplete dummy data")
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

def remote_node_dummy():
    """
    Remote Node using dummy data for testing and development.
    """
    rospy.init_node('remote_node_dummy', anonymous=True)

    # Publishers for the topics
    mode_pub = rospy.Publisher('mode', String, queue_size=10)
    arming_pub = rospy.Publisher('arming', Bool, queue_size=10)
    throttle_pub = rospy.Publisher('throttle_remote', Int16, queue_size=10)
    yaw_pub = rospy.Publisher('yaw_remote', Int16, queue_size=10)

    # Loop to generate and process dummy data
    rate = rospy.Rate(50)  # 50 Hz
    while not rospy.is_shutdown():
        try:
            # Generate dummy data
            data = generate_dummy_data()
            mode, arming, throttle, yaw = process_remote_data(data)

            if mode is not None:
                mode_pub.publish(mode)
                arming_pub.publish(arming)
                throttle_pub.publish(throttle)
                yaw_pub.publish(yaw)

        except Exception as e:
            rospy.logerr(f"Error processing dummy data: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        remote_node_dummy()
    except rospy.ROSInterruptException:
        pass
