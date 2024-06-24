import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import math

class MotionController:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/amr/cmd_vel', Twist, queue_size=10)
        self.path_sub = rospy.Subscriber('/amr/path', Path, self.path_callback)
        self.path = None
    
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
                
                self.cmd_vel_pub.publish(cmd_vel)
                rospy.sleep(0.1)
                
                current_position = rospy.wait_for_message('/amr/pose', PoseWithCovarianceStamped).pose.pose.position
        
        self.stop()
    
    def stop(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
    
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
