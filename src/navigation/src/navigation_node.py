#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool, Int16
from mavros_msgs.msg import OverrideRCIn, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode

class NavigationNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('navigation_node', anonymous=True)

        # Publishers
        self.override_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        self.position_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', PositionTarget, queue_size=10)

        # Service proxies for arming and mode
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool, persistent=True)
        self.mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode, persistent=True)

        # Subscribers
        rospy.Subscriber('/throttle_remote', Int16, self.throttle_callback)
        rospy.Subscriber('/yaw_remote', Int16, self.yaw_callback)
        rospy.Subscriber('/mode', String, self.mode_callback)
        rospy.Subscriber('/arming', Bool, self.arming_callback)

        # Default control values
        self.throttle = 1500  # Neutral throttle
        self.yaw = 1500  # Neutral yaw
        self.mode = "neutral"  # Default mode
        self.is_armed = False  # Disarmed by default

    def throttle_callback(self, msg):
        """Update throttle from remote."""
        self.throttle = msg.data

    def yaw_callback(self, msg):
        """Update yaw from remote."""
        self.yaw = msg.data

    def mode_callback(self, msg):
        """Update mode from remote."""
        new_mode = msg.data
        if new_mode != self.mode:
            self.change_mode(new_mode)
        self.mode = new_mode

    def arming_callback(self, msg):
        """Update arming state from remote."""
        should_arm = msg.data
        if should_arm != self.is_armed:
            self.change_arming(should_arm)
        self.is_armed = should_arm

    def change_mode(self, new_mode):
        """Change flight mode on Pixhawk."""
        try:
            response = self.mode_client(custom_mode=new_mode)
            if response.mode_sent:
                rospy.loginfo(f"Mode changed to {new_mode} successfully.")
            else:
                rospy.logwarn(f"Failed to change mode to {new_mode}.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def change_arming(self, should_arm):
        """Arm or disarm the vehicle."""
        try:
            response = self.arming_client(value=should_arm)
            if response.success:
                state = "armed" if should_arm else "disarmed"
                rospy.loginfo(f"Vehicle {state} successfully.")
            else:
                rospy.logwarn("Failed to change arming state.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def send_control(self):
        """Send control commands to Pixhawk via MAVROS."""
        override_msg = OverrideRCIn()
        if self.mode == "manual":
            # Set control values for manual mode
            override_msg.channels[1] = self.yaw  # Yaw (Channel 0)
            override_msg.channels[3] = self.throttle  # Throttle (Channel 2)
            # Set other control channels to neutral (1500)
            override_msg.channels[0] = 1500  # Pitch (Channel 1)
            override_msg.channels[2] = 1500  # Roll (Channel 3)
            override_msg.channels[4] = 1000  # Invalid for manual mode
            override_msg.channels[5] = 1000  # Invalid for manual mode
            override_msg.channels[6] = 1000  # Invalid for manual mode
            override_msg.channels[7] = 1000  # Invalid for manual mode
            self.override_pub.publish(override_msg)

    def run(self):
        """Main loop for navigation node."""
        rate = rospy.Rate(20)  # 20 Hz
        while not rospy.is_shutdown():
            self.send_control()
            rate.sleep()

if __name__ == '__main__':
    try:
        nav_node = NavigationNode()
        nav_node.run()
    except rospy.ROSInterruptException:
        pass
