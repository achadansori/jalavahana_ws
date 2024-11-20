#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, Int16MultiArray
from mavros_msgs.msg import State

class PIDController:
    def __init__(self, kp, ki, kd, min_output, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        self.reset()
    
    def reset(self):
        self.last_error = 0
        self.integral = 0
        self.last_time = rospy.get_time()
    
    def compute(self, error):
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            return 0
            
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative
        
        # Calculate total output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(self.min_output, min(self.max_output, output))
        
        # Update state
        self.last_error = error
        self.last_time = current_time
        
        return output

class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node', anonymous=True)
        
        # PID Controllers
        self.distance_pid = PIDController(kp=0.5, ki=0.1, kd=0.2, min_output=-1.0, max_output=1.0)
        self.heading_pid = PIDController(kp=0.8, ki=0.1, kd=0.3, min_output=-1.0, max_output=1.0)
        
        # Navigation parameters
        self.target_lat = 0.0
        self.target_lon = 0.0
        self.is_target_set = False
        self.auto_mode = False
        self.auto_threshold = 1500  # Threshold untuk mode auto
        
        # Remote control data
        self.remote_data = [1500] * 10
        
        # Subscribers
        self.gps_sub = rospy.Subscriber('/mavros/global_position/global', 
                                      NavSatFix, 
                                      self.gps_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', 
                                        State, 
                                        self.state_callback)
        self.remote_sub = rospy.Subscriber('/remote/raw_data', 
                                         Int16MultiArray, 
                                         self.remote_callback)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', 
                                         Twist, 
                                         queue_size=10)
        self.target_pub = rospy.Publisher('/navigation/target_position', 
                                        NavSatFix, 
                                        queue_size=10)
        self.distance_pub = rospy.Publisher('/navigation/distance_to_target', 
                                          Float64, 
                                          queue_size=10)
        self.heading_pub = rospy.Publisher('/navigation/heading_error', 
                                         Float64, 
                                         queue_size=10)
        self.mode_pub = rospy.Publisher('/navigation/current_mode', 
                                      Bool, 
                                      queue_size=10)
        
        # Control parameters
        self.max_linear_speed = 1.0
        self.max_angular_speed = 0.5
        self.target_reach_threshold = 3.0
        
        # Initialize variables
        self.current_position = NavSatFix()
        self.current_state = State()
        
        # Rate
        self.rate = rospy.Rate(20)
    
    def map_range(self, value, in_min, in_max, out_min, out_max):
        """Map value from input range to output range"""
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def remote_callback(self, msg):
        """Handle remote control data and mode switching"""
        self.remote_data = msg.data
        
        # Update mode based on channel 6 (index 5)
        new_mode = self.remote_data[5] > self.auto_threshold
        if new_mode != self.auto_mode:
            self.auto_mode = new_mode
            self.mode_pub.publish(Bool(self.auto_mode))
            rospy.loginfo(f"Switching to {'Autonomous' if self.auto_mode else 'Manual'} Mode")
            
            if self.auto_mode:
                self.distance_pid.reset()
                self.heading_pid.reset()
    
    def state_callback(self, msg):
        self.current_state = msg
    
    def gps_callback(self, msg):
        self.current_position = msg
        if self.is_target_set and self.auto_mode:
            self.navigate_to_target()
    
    def process_manual_control(self):
        """Process manual control from remote"""
        if not self.remote_data:
            return
            
        cmd_vel = Twist()
        
        # Map yaw dari remote (CH1) ke angular velocity
        yaw = self.map_range(
            self.remote_data[0],  # Channel 1 untuk yaw
            1000, 2000,
            -self.max_angular_speed, self.max_angular_speed
        )
        
        # Map throttle dari remote (CH3) ke linear velocity
        throttle = self.map_range(
            self.remote_data[2],  # Channel 3 untuk throttle
            1000, 2000,
            -self.max_linear_speed, self.max_linear_speed
        )
        
        cmd_vel.linear.x = throttle
        cmd_vel.angular.z = yaw
        
        self.cmd_vel_pub.publish(cmd_vel)
    
    def navigate_to_target(self):
        """Navigate to target using PID control"""
        if not self.is_target_set:
            return
            
        distance, bearing = self.get_distance_and_bearing()
        
        self.distance_pub.publish(Float64(distance))
        self.heading_pub.publish(Float64(math.degrees(bearing)))
        
        if distance < self.target_reach_threshold:
            rospy.loginfo("Target reached!")
            self.is_target_set = False
            return
        
        # Calculate PID outputs
        distance_output = self.distance_pid.compute(distance)
        heading_output = self.heading_pid.compute(bearing)
        
        cmd_vel = Twist()
        cmd_vel.linear.x = distance_output * self.max_linear_speed
        cmd_vel.angular.z = heading_output * self.max_angular_speed
        
        self.cmd_vel_pub.publish(cmd_vel)
    
    def get_distance_and_bearing(self):
        """Calculate distance and bearing to target"""
        lat1 = math.radians(self.current_position.latitude)
        lon1 = math.radians(self.current_position.longitude)
        lat2 = math.radians(self.target_lat)
        lon2 = math.radians(self.target_lon)
        
        dlon = lon2 - lon1
        
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - \
            math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        
        bearing = math.atan2(y, x)
        
        a = math.sin((lat2-lat1)/2)**2 + \
            math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6371000 * c  # Earth's radius in meters
        
        return distance, bearing
    
    def set_target(self, lat, lon):
        """Set target GPS coordinates"""
        self.target_lat = lat
        self.target_lon = lon
        self.is_target_set = True
        
        target_msg = NavSatFix()
        target_msg.latitude = lat
        target_msg.longitude = lon
        self.target_pub.publish(target_msg)
        
        rospy.loginfo(f"New target set: Lat {lat}, Lon {lon}")
    
    def run(self):
        """Main run loop"""
        while not rospy.is_shutdown():
            if not self.current_state.connected:
                rospy.logwarn_throttle(5, "Not connected to FCU!")
                continue
            
            # Process control based on mode
            if not self.auto_mode:
                self.process_manual_control()
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        nav_node = NavigationNode()
        
        # Set target GPS (sesuaikan dengan kebutuhan)
        nav_node.set_target(-6.914744, 107.609810)
        
        nav_node.run()
    except rospy.ROSInterruptException:
        pass