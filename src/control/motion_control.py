import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import math
import serial
import time

class MotionController:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/amr/cmd_vel', Twist, queue_size=10)
        self.path_sub = rospy.Subscriber('/amr/path', Path, self.path_callback)
        self.path = None
        
        # Set up serial connection to the Arduino
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        time.sleep(2)  # Wait for the serial connection to initialize
    
    def path_callback(self, msg):
        self.path = msg
    
    def follow_path(self):
        if self.path is None:
            return
        
        for pose in self.path.poses:
            goal_position = pose.pose.position
            current_position = rospy.wait_for_message('/amr/pose', PoseWithCovarianceStamped).pose.pose.position
            
            while self.euclidean_distance(current_position, goal_position) > 0.1:
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.2
                
                goal_yaw = math.atan2(goal_position.y - current_position.y, goal_position.x - current_position.x)
                current_yaw = rospy.wait_for_message('/amr/pose', PoseWithCovarianceStamped).pose.pose.orientation
                current_yaw = self.quaternion_to_euler(current_yaw)[-1]
                
                angular_error = goal_yaw - current_yaw
                cmd_vel.angular.z = 0.5 * angular_error
                
                # Convert cmd_vel to stepper speeds and send to Arduino
                stepper_speeds = self.cmd_vel_to_stepper_speeds(cmd_vel)
                self.send_speeds_to_arduino(stepper_speeds)
                
                rospy.sleep(0.1)
                
                current_position = rospy.wait_for_message('/amr/pose', PoseWithCovarianceStamped).pose.pose.position
        
        self.stop()
    
    def stop(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        
        # Convert cmd_vel to stepper speeds and send to Arduino
        stepper_speeds = self.cmd_vel_to_stepper_speeds(cmd_vel)
        self.send_speeds_to_arduino(stepper_speeds)
    
    def euclidean_distance(self, p1, p2):
        return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
    
    def quaternion_to_euler(self, quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [roll, pitch, yaw]
    
    def cmd_vel_to_stepper_speeds(self, cmd_vel):
        # Implement the conversion from cmd_vel to stepper speeds based on your AMR's kinematics
        # This is just a placeholder example
        stepper_speeds = [
            int(cmd_vel.linear.x * 100),
            int(cmd_vel.linear.x * 100),
            int(cmd_vel.angular.z * 100),
            int(cmd_vel.angular.z * 100)
        ]
        return stepper_speeds
    
    def send_speeds_to_arduino(self, stepper_speeds):
        speeds_str = " ".join(str(speed) for speed in stepper_speeds) + " 100\n"
        self.ser.write(speeds_str.encode())
        
        # Wait for acknowledgment and speed values from Arduino
        while True:
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode().strip()
                if response.startswith("SPEEDS "):
                    received_speeds = response.split()[1:]
                    errors = self.calculate_error(stepper_speeds, received_speeds)
                    rospy.loginfo("Communication error values:")
                    rospy.loginfo(f"Stepper 1: {errors[0]} sps")
                    rospy.loginfo(f"Stepper 2: {errors[1]} sps")
                    rospy.loginfo(f"Stepper 3: {errors[2]} sps")
                    rospy.loginfo(f"Stepper 4: {errors[3]} sps")
                    break
                elif response == "ERROR":
                    rospy.logerr("Arduino detected invalid speed values. Please try again.")
                    break
                else:
                    rospy.logwarn("Unknown response from Arduino. Please check the serial connection.")
                    break
    
    def calculate_error(self, sent_speeds, received_speeds):
        errors = []
        for sent, received in zip(sent_speeds, received_speeds):
            error = abs(int(sent) - int(received))
            errors.append(error)
        return errors
